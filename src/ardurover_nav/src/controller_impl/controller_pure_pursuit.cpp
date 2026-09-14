#include "ardurover_nav/controller_impl/controller_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ardurover_nav {

ControllerPurePursuit::ControllerPurePursuit(rclcpp::Node& node, std::vector<Waypoint> path)
    : ArduroverController(node, path) {
    maxSpeed_ = node.declare_parameter("max_speed", 1.0);
    lookaheadMin_ = node.declare_parameter("lookahead_min", 0.75);
    lookaheadMax_ = node.declare_parameter("lookahead_max", 2.5);
    lookaheadBase_ = node.declare_parameter("lookahead_base", 0.6);
    lookaheadKv_ = node.declare_parameter("lookahead_k_v", 1.0);
    lookaheadKKappa_ = node.declare_parameter("lookahead_k_kappa", 0.4);
    lookaheadMaxDTheta_ = node.declare_parameter("lookahead_max_dtheta", 0.85);
    maxYawRate_ = node.declare_parameter("max_yaw_rate", 1.0);
    latAccelMax_ = node.declare_parameter("lat_accel_max", 1.0);
    decel_ = node.declare_parameter("decel", 0.8);
    pivotAngle_ = node.declare_parameter("pivot_angle", 0.8);
    pivotSpeed_ = node.declare_parameter("pivot_speed", 0.15);
    reverseAngle_ = node.declare_parameter("reverse_angle", 2.0);
    goalTolerance_ = node.declare_parameter("goal_tolerance", 0.25);
    yawRateSign_ = node.declare_parameter("yaw_rate_sign", 1.0);
    omegaSpeedFloor_ = node.declare_parameter("omega_speed_floor", 0.2);
    omegaTau_ = node.declare_parameter("omega_tau", 0.2);
    curvaturePreview_ = node.declare_parameter("curvature_preview", 3.0);
    stuckSpeedEps_ = node.declare_parameter("stuck_speed_eps", 0.08);
    stuckTicksLimit_ = static_cast<int>(node.declare_parameter("stuck_ticks_limit", 20));

    BuildTrack(path_);

    RCLCPP_INFO_STREAM(
        node_.get_logger(), "ControllerPurePursuit ready: " << track_.size() << " track points, length "
                                                            << pathLength_ << " m"
    );
}

void ControllerPurePursuit::BuildTrack(const std::vector<Waypoint>& path) {
    track_.clear();
    track_.reserve(path.size());
    for (const auto& wp : path) {
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

void ControllerPurePursuit::ConsiderSegment(Projection& best, size_t i, double px, double py) const {
    const double ax = track_[i].x;
    const double ay = track_[i].y;
    const double bx = track_[i + 1].x;
    const double by = track_[i + 1].y;
    const double abx = bx - ax;
    const double aby = by - ay;
    const double length2 = abx * abx + aby * aby;
    double t = 0.0;
    if (length2 > 1e-12) {
        t = std::clamp(((px - ax) * abx + (py - ay) * aby) / length2, 0.0, 1.0);
    }
    const double cx = ax + t * abx;
    const double cy = ay + t * aby;
    const double dist = std::hypot(px - cx, py - cy);
    if (dist < best.dist) {
        best.dist = dist;
        best.segIndex = i;
        best.t = t;
        best.x = cx;
        best.y = cy;
        best.s = track_[i].s + t * (track_[i + 1].s - track_[i].s);
    }
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

    for (size_t i = i_start; i <= seg_end; ++i) {
        ConsiderSegment(best, i, px, py);
    }
    if (!std::isfinite(best.dist)) {
        for (size_t i = 0; i <= last_seg_idx; ++i) {
            ConsiderSegment(best, i, px, py);
        }
    }
    return best;
}

ControllerPurePursuit::PathPoint ControllerPurePursuit::PointAtArcLength(double s) const {
    s = std::clamp(s, 0.0, pathLength_);
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

ControllerPurePursuit::PathPoint ControllerPurePursuit::LookaheadPoint(
    double s0, double ld, double px, double py
) const {
    PathPoint prev = PointAtArcLength(s0);
    double prev_dist = std::hypot(prev.x - px, prev.y - py);
    if (prev_dist >= ld || s0 >= pathLength_ - 1e-6) {
        return prev;
    }

    constexpr double kStep = 0.05;
    for (double s = s0 + kStep; s < pathLength_; s += kStep) {
        const PathPoint cur = PointAtArcLength(s);
        const double dist = std::hypot(cur.x - px, cur.y - py);
        if (dist >= ld) {
            const double denom = dist - prev_dist;
            const double t = (denom > 1e-9) ? std::clamp((ld - prev_dist) / denom, 0.0, 1.0) : 1.0;
            PathPoint out;
            out.x = prev.x + t * (cur.x - prev.x);
            out.y = prev.y + t * (cur.y - prev.y);
            out.s = prev.s + t * (cur.s - prev.s);
            return out;
        }
        prev = cur;
        prev_dist = dist;
    }
    return track_.back();
}

double ControllerPurePursuit::HeadingLimitedLookahead(double s, double ld_max, double max_dtheta) const {
    if (ld_max <= 1e-6 || track_.size() < 2) {
        return ld_max;
    }
    const double h0 = HeadingAt(s);
    constexpr double kStep = 0.08;
    double last = 0.0;
    const double s_stop = std::min(s + ld_max, pathLength_);
    for (double ss = s + kStep; ss <= s_stop; ss += kStep) {
        if (std::abs(WrapAngle(HeadingAt(ss) - h0)) > max_dtheta) {
            return std::max(last, 0.55);
        }
        last = ss - s;
    }
    return ld_max;
}

double ControllerPurePursuit::RawHeadingAt(double s) const {
    if (track_.size() < 2) {
        return 0.0;
    }
    s = std::clamp(s, 0.0, pathLength_);
    size_t i = 0;
    while (i + 1 < track_.size() && track_[i + 1].s < s) {
        ++i;
    }
    if (i + 1 >= track_.size()) {
        i = track_.size() - 2;
    }
    return std::atan2(track_[i + 1].y - track_[i].y, track_[i + 1].x - track_[i].x);
}

double ControllerPurePursuit::HeadingAt(double s) const {
    constexpr double kHalf = 0.3;
    const PathPoint a = PointAtArcLength(s - kHalf);
    const PathPoint b = PointAtArcLength(s + kHalf);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (dx * dx + dy * dy < 1e-8) {
        return RawHeadingAt(s);
    }
    return std::atan2(dy, dx);
}

double ControllerPurePursuit::PathCurvatureNear(double s, double preview) const {
    const double s0 = std::clamp(s, 0.0, pathLength_);
    const double s1 = std::clamp(s + preview, 0.0, pathLength_);
    if (s1 - s0 < 0.3 || track_.size() < 3) {
        return 0.0;
    }

    double max_abs_kappa = 0.0;
    constexpr double kStep = 0.8;
    for (double sa = s0; sa + 0.4 < s1; sa += kStep) {
        const double sb = std::min(sa + kStep, s1);
        const double dtheta = WrapAngle(HeadingAt(sb) - HeadingAt(sa));
        const double kappa = std::abs(dtheta / (sb - sa));
        max_abs_kappa = std::max(max_abs_kappa, kappa);
    }
    return max_abs_kappa;
}

double ControllerPurePursuit::WrapAngle(double a) const {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

void ControllerPurePursuit::Control(const nav_msgs::msg::Odometry& odom) {
    if (goalReached_) {
        PublishStop();
        return;
    }

    const double px = odom.pose.pose.position.x;
    const double py = odom.pose.pose.position.y;
    const double yaw = yaw_from_quat(odom.pose.pose.orientation);

    const Projection proj = ProjectOntoPath(px, py);
    lastSeg_ = proj.segIndex;

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

    const double vx = odom.twist.twist.linear.x;
    const double vy = odom.twist.twist.linear.y;
    const double speed_meas = std::hypot(vx, vy);
    const double dt = TickDt(odom.header.stamp);
    // Upcoming curvature slows the rover; local curvature (not a 3 m preview of path
    // noise) is what should shrink lookahead, otherwise straights weave.
    const double abs_kappa = PathCurvatureNear(proj.s, curvaturePreview_);
    const double kappa_ld = PathCurvatureNear(proj.s, 1.0);

    const double speed_for_ld = std::max(speed_meas, 0.2);
    double lookahead = lookaheadBase_ + lookaheadKv_ * speed_for_ld;
    lookahead /= (1.0 + lookaheadKKappa_ * kappa_ld);
    lookahead = std::clamp(lookahead, lookaheadMin_, lookaheadMax_);
    lookahead = std::min(lookahead, HeadingLimitedLookahead(proj.s, lookahead, lookaheadMaxDTheta_));
    lookahead = std::max(lookahead, 0.55);
    lookahead = std::min(lookahead, std::max(s_remain, 0.55));

    // Along-track target is stable on a wiggly recorded polyline; use the circle
    // intersection only when already off the path.
    const PathPoint target = (proj.dist > 0.6)
        ? LookaheadPoint(proj.s, lookahead, px, py)
        : PointAtArcLength(proj.s + lookahead);

    const double bearing = std::atan2(target.y - py, target.x - px);
    double alpha = WrapAngle(bearing - yaw);

    // Path 2 records reverse segments; chase those with the rear instead of U-turning.
    const double heading_err = WrapAngle(RawHeadingAt(proj.s) - yaw);
    const bool reverse = std::abs(heading_err) > reverseAngle_;
    if (reverse) {
        alpha = WrapAngle(alpha - std::copysign(M_PI, alpha));
    }

    const double kappa_pp = (2.0 * std::sin(alpha)) / std::max(lookahead, 1e-3);

    double v_cmd = maxSpeed_;
    if (abs_kappa > 1e-4) {
        v_cmd = std::min(v_cmd, std::sqrt(latAccelMax_ / abs_kappa));
    }

    // Pivot in place when the lookahead bearing is large so the rover does not cut the corner.
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

    const bool recovering = stuckTicks_ >= stuckTicksLimit_;
    if (stuckTicks_ >= stuckTicksLimit_ * 3) {
        const bool reverse_nudge = (stuckTicks_ / 10) % 2 == 0;
        v_cmd = reverse_nudge ? -0.35 : 0.2;
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000, "Stuck recovery: rock %s (alpha=%.2f)",
            reverse_nudge ? "reverse" : "forward", alpha
        );
    } else if (recovering) {
        v_cmd = 0.0;
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000, "Stuck recovery: pivoting in place (alpha=%.2f)", alpha
        );
    }

    v_cmd = std::clamp(v_cmd, -0.4, maxSpeed_);
    if (reverse && !recovering) {
        const double mag = (s_remain < 1.0) ? std::abs(v_cmd) : std::max(std::abs(v_cmd), pivotSpeed_);
        v_cmd = -mag;
    }

    const double v_for_omega = std::max(std::abs(v_cmd), omegaSpeedFloor_);
    double omega = std::clamp(v_for_omega * kappa_pp, -maxYawRate_, maxYawRate_);
    if (misaligned || recovering) {
        omega = (alpha >= 0.0 ? 1.0 : -1.0) * maxYawRate_;
        omegaFilt_ = omega;
        haveOmegaFilt_ = true;
    } else if (!haveOmegaFilt_) {
        omegaFilt_ = omega;
        haveOmegaFilt_ = true;
    } else {
        const double alpha_f = dt / (omegaTau_ + dt);
        omegaFilt_ += alpha_f * (omega - omegaFilt_);
        omega = omegaFilt_;
    }
    omega *= yawRateSign_;

    PublishTwist(v_cmd, omega);

    RCLCPP_INFO_STREAM_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "v=" << v_cmd << " w=" << omega << " alpha=" << alpha << " Ld=" << lookahead << " cte=" << proj.dist
             << " s=" << proj.s << "/" << pathLength_ << " kpath=" << abs_kappa << " spd=" << speed_meas
             << " rev=" << reverse
    );
}

}  // namespace ardurover_nav
