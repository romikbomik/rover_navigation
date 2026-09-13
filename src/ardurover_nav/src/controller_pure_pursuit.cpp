#include "ardurover_nav/controller_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ardurover_nav {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double a) {
    while (a > kPi) {
        a -= 2.0 * kPi;
    }
    while (a < -kPi) {
        a += 2.0 * kPi;
    }
    return a;
}

double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

}  // namespace

ControllerPurePursuit::ControllerPurePursuit(rclcpp::Node& node, std::vector<Waypoint> path)
    : ArduroverController(node, path) {
    maxSpeed_ = node.declare_parameter("max_speed", 1.0);
    lookaheadMin_ = node.declare_parameter("lookahead_min", 1.5);
    lookaheadMax_ = node.declare_parameter("lookahead_max", 4.0);
    lookaheadBase_ = node.declare_parameter("lookahead_base", 1.2);
    lookaheadKv_ = node.declare_parameter("lookahead_k_v", 1.0);
    maxYawRate_ = node.declare_parameter("max_yaw_rate", 1.0);
    latAccelMax_ = node.declare_parameter("lat_accel_max", 1.5);
    decel_ = node.declare_parameter("decel", 0.8);
    pivotAngle_ = node.declare_parameter("pivot_angle", 0.8);
    pivotSpeed_ = node.declare_parameter("pivot_speed", 0.15);
    goalTolerance_ = node.declare_parameter("goal_tolerance", 0.25);
    yawRateSign_ = node.declare_parameter("yaw_rate_sign", 1.0);
    omegaSpeedFloor_ = node.declare_parameter("omega_speed_floor", 0.35);
    curvaturePreview_ = node.declare_parameter("curvature_preview", 2.0);
    stuckSpeedEps_ = node.declare_parameter("stuck_speed_eps", 0.08);
    stuckTicksLimit_ = static_cast<int>(node.declare_parameter("stuck_ticks_limit", 20));

    BuildTrack(path_);
    lookaheadPub_ = node_.create_publisher<visualization_msgs::msg::Marker>("/lookahead_marker", 1);

    RCLCPP_INFO_STREAM(
        node_.get_logger(), "ControllerPurePursuit ready: " << track_.size() << " track points, length "
                                                            << pathLength_ << " m"
    );
}

void ControllerPurePursuit::BuildTrack(const std::vector<Waypoint>& path) {
    constexpr double kMinSpacing = 0.05;
    track_.clear();
    track_.reserve(path.size());
    for (const auto& wp : path) {
        if (!track_.empty()) {
            const double dx = wp.x - track_.back().x;
            const double dy = wp.y - track_.back().y;
            if (dx * dx + dy * dy < kMinSpacing * kMinSpacing) {
                continue;
            }
        }
        PathPoint p;
        p.x = wp.x;
        p.y = wp.y;
        p.s = track_.empty() ? 0.0 : track_.back().s + std::hypot(wp.x - track_.back().x, wp.y - track_.back().y);
        track_.push_back(p);
    }

    if (track_.empty()) {
        throw std::runtime_error("Path is empty");
    }
    pathLength_ = track_.back().s;
}

ControllerPurePursuit::Projection ControllerPurePursuit::ProjectOntoPath(double px, double py) const {
    Projection best;
    best.dist = std::numeric_limits<double>::infinity();

    if (track_.size() < 2) {
        best.x = track_.front().x;
        best.y = track_.front().y;
        best.s = 0.0;
        best.dist = std::hypot(px - best.x, py - best.y);
        return best;
    }

    constexpr double kLookBack = 2.0;
    constexpr double kLookAhead = 10.0;
    const double s_ref = track_[std::min(lastSeg_, track_.size() - 1)].s;
    size_t i_start = lastSeg_;
    size_t i_end = lastSeg_;
    while (i_start > 0 && track_[i_start].s > s_ref - kLookBack) {
        --i_start;
    }
    while (i_end + 1 < track_.size() && track_[i_end].s < s_ref + kLookAhead) {
        ++i_end;
    }
    const size_t last_seg_idx = track_.size() - 2;
    const size_t seg_end = std::min(i_end, last_seg_idx);

    auto consider = [&](size_t i) {
        const double ax = track_[i].x;
        const double ay = track_[i].y;
        const double bx = track_[i + 1].x;
        const double by = track_[i + 1].y;
        const double abx = bx - ax;
        const double aby = by - ay;
        const double length2 = abx * abx + aby * aby;
        double t = 0.0;
        if (length2 > 1e-12) {
            t = clamp(((px - ax) * abx + (py - ay) * aby) / length2, 0.0, 1.0);
        }
        const double cx = ax + t * abx;
        const double cy = ay + t * aby;
        const double dist = std::hypot(px - cx, py - cy);
        if (dist < best.dist) {
            best.dist = dist;
            best.seg_index = i;
            best.t = t;
            best.x = cx;
            best.y = cy;
            best.s = track_[i].s + t * (track_[i + 1].s - track_[i].s);
        }
    };

    for (size_t i = i_start; i <= seg_end; ++i) {
        consider(i);
    }

    if (!std::isfinite(best.dist)) {
        for (size_t i = 0; i <= last_seg_idx; ++i) {
            consider(i);
        }
    }

    return best;
}

ControllerPurePursuit::PathPoint ControllerPurePursuit::PointAtArcLength(double s) const {
    s = clamp(s, 0.0, pathLength_);
    if (track_.size() == 1) {
        return track_.front();
    }
    size_t i = 0;
    while (i + 1 < track_.size() && track_[i + 1].s < s) {
        ++i;
    }
    if (i + 1 >= track_.size()) {
        return track_.back();
    }
    const double seg_len = track_[i + 1].s - track_[i].s;
    const double t = (seg_len < 1e-12) ? 0.0 : (s - track_[i].s) / seg_len;
    PathPoint p;
    p.x = track_[i].x + t * (track_[i + 1].x - track_[i].x);
    p.y = track_[i].y + t * (track_[i + 1].y - track_[i].y);
    p.s = s;
    return p;
}

double ControllerPurePursuit::PathCurvatureNear(double s) const {
    const double s0 = clamp(s, 0.0, pathLength_);
    const double s1 = clamp(s + curvaturePreview_, 0.0, pathLength_);
    if (s1 - s0 < 0.3 || track_.size() < 3) {
        return 0.0;
    }

    auto heading_at = [this](double ss) {
        ss = clamp(ss, 0.0, pathLength_);
        size_t i = 0;
        while (i + 1 < track_.size() && track_[i + 1].s < ss) {
            ++i;
        }
        if (i + 1 >= track_.size()) {
            i = track_.size() - 2;
        }
        return std::atan2(track_[i + 1].y - track_[i].y, track_[i + 1].x - track_[i].x);
    };

    double max_abs_kappa = 0.0;
    constexpr double kStep = 0.4;
    for (double sa = s0; sa + 0.25 < s1; sa += kStep) {
        const double sb = std::min(sa + kStep, s1);
        const double dtheta = wrap_angle(heading_at(sb) - heading_at(sa));
        const double kappa = std::abs(dtheta / (sb - sa));
        max_abs_kappa = std::max(max_abs_kappa, kappa);
    }
    return max_abs_kappa;
}

void ControllerPurePursuit::PublishLookaheadMarker(double x, double y) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_.get_clock()->now();
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.3;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color.r = 0.1f;
    marker.color.g = 0.4f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    lookaheadPub_->publish(marker);
}

void ControllerPurePursuit::Control(const nav_msgs::msg::Odometry& odometry) {
    if (goalReached_) {
        PublishStop();
        return;
    }

    const double px = odometry.pose.pose.position.x;
    const double py = odometry.pose.pose.position.y;
    const double yaw = yaw_from_quat(odometry.pose.pose.orientation);

    const Projection proj = ProjectOntoPath(px, py);
    lastSeg_ = proj.seg_index;

    const double s_remain = pathLength_ - proj.s;
    const double dist_to_goal = std::hypot(px - track_.back().x, py - track_.back().y);

    if ((s_remain < 0.5 && dist_to_goal < goalTolerance_) || s_remain < 0.15) {
        goalReached_ = true;
        RCLCPP_INFO(
            node_.get_logger(), "Goal reached (s_remain=%.2f m, dist=%.2f m) — holding stop", s_remain, dist_to_goal
        );
        PublishStop();
        return;
    }

    const double vx = odometry.twist.twist.linear.x;
    const double vy = odometry.twist.twist.linear.y;
    const double speed_meas = std::hypot(vx, vy);

    const double speed_for_ld = std::max(speed_meas, 0.4);
    const double Ld = clamp(lookaheadBase_ + lookaheadKv_ * speed_for_ld, lookaheadMin_, lookaheadMax_);
    const PathPoint target = PointAtArcLength(proj.s + Ld);
    PublishLookaheadMarker(target.x, target.y);

    const double bearing = std::atan2(target.y - py, target.x - px);
    const double alpha = wrap_angle(bearing - yaw);
    const double kappa_pp = (2.0 * std::sin(alpha)) / std::max(Ld, 1e-3);
    const double abs_kappa = PathCurvatureNear(proj.s);

    double v_cmd = maxSpeed_;

    if (abs_kappa > 1e-4) {
        const double kappa_eff = std::min(abs_kappa, 2.0);
        v_cmd = std::min(v_cmd, std::sqrt(latAccelMax_ / kappa_eff));
    }

    const bool misaligned = std::abs(alpha) > pivotAngle_;
    if (misaligned) {
        v_cmd = std::min(v_cmd, pivotSpeed_);
    }

    if (s_remain > 1e-3) {
        v_cmd = std::min(v_cmd, std::sqrt(2.0 * decel_ * s_remain));
    } else {
        v_cmd = 0.0;
    }

    if (speed_meas < stuckSpeedEps_) {
        ++stuckTicks_;
    } else {
        stuckTicks_ = 0;
    }

    if (stuckTicks_ >= stuckTicksLimit_ * 3) {
        const bool reverse_nudge = (stuckTicks_ / 10) % 2 == 0;
        v_cmd = reverse_nudge ? -0.35 : 0.2;
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000, "Stuck recovery: rock %s (alpha=%.2f)",
            reverse_nudge ? "reverse" : "forward", alpha
        );
    } else if (stuckTicks_ >= stuckTicksLimit_) {
        v_cmd = 0.0;
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000, "Stuck recovery: pivoting in place (alpha=%.2f)", alpha
        );
    }

    v_cmd = clamp(v_cmd, -0.4, maxSpeed_);

    const double v_for_omega = std::max(std::abs(v_cmd), omegaSpeedFloor_);
    double omega = clamp(v_for_omega * kappa_pp, -maxYawRate_, maxYawRate_);
    if (misaligned || stuckTicks_ >= stuckTicksLimit_) {
        omega = (alpha >= 0.0 ? 1.0 : -1.0) * maxYawRate_;
    }
    omega *= yawRateSign_;

    PublishTwist(v_cmd, omega);

    RCLCPP_INFO_STREAM_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "v=" << v_cmd << " w=" << omega << " alpha=" << alpha << " Ld=" << Ld << " cte=" << proj.dist
             << " s=" << proj.s << "/" << pathLength_ << " kpath=" << abs_kappa << " spd=" << speed_meas
    );
}

}  // namespace ardurover_nav
