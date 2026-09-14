#include "ardurover_nav/controller_impl/controller_stanley.hpp"

#include <algorithm>
#include <cmath>

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

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

}  // namespace

ControllerStanley::ControllerStanley(rclcpp::Node& node, std::vector<Waypoint> path)
    : ArduroverController(node, path), refPath_(path_) {
    kGain_ = node.declare_parameter("stanley_k", 2.0);
    kSoft_ = node.declare_parameter("stanley_k_soft", 1.0);
    maxSpeed_ = node.declare_parameter("max_speed", 1.0);
    maxYawRate_ = node.declare_parameter("max_yaw_rate", 1.0);
    pivotAngle_ = node.declare_parameter("pivot_angle", 1.0);
    pivotSpeed_ = node.declare_parameter("pivot_speed", 0.15);
    reverseAngle_ = node.declare_parameter("reverse_angle", 2.0);
    slowRadius_ = node.declare_parameter("slow_radius", 1.5);
    goalTolerance_ = node.declare_parameter("goal_tolerance", 0.25);
    stuckSpeedEps_ = node.declare_parameter("stuck_speed_eps", 0.08);
    stuckTicksLimit_ = static_cast<int>(node.declare_parameter("stuck_ticks_limit", 20));

    RCLCPP_WARN(node_.get_logger(), "ControllerStanley is experimental / work in progress");
    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "ControllerStanley ready: path length " << refPath_.Length() << " m, k/k_soft = " << kGain_ << "/" << kSoft_
    );
}

void ControllerStanley::Control(const nav_msgs::msg::Odometry& odom) {
    if (goalReached_) {
        PublishStop();
        return;
    }

    const double px = odom.pose.pose.position.x;
    const double py = odom.pose.pose.position.y;
    const double yaw = yaw_from_quat(odom.pose.pose.orientation);

    const PathProjection proj = refPath_.Project(px, py);
    const double s_remain = refPath_.RemainingLength(proj.s);
    const double dist_to_goal = std::hypot(px - path_.back().x, py - path_.back().y);

    // Remaining-only at goal_tolerance (0.25 m) misses the recorded reverse tail
    // on path 2 (~0.32 m). If the rover does not actually reverse onto that tail,
    // s_remain plateaus above the threshold and Stanley keeps commanding motion
    // past the end. Latch on remaining arc, or on Euclidean distance once we have
    // followed most of the path — same idea as pure pursuit / the scorer.
    constexpr double kRemainStop = 0.5;
    const bool near_end = proj.s >= 0.90 * refPath_.Length();
    if (s_remain < kRemainStop || (near_end && dist_to_goal < goalTolerance_)) {
        goalReached_ = true;
        PublishStop();
        RCLCPP_INFO(
            node_.get_logger(), "Goal reached (s_remain=%.2f m, dist=%.2f m) — holding stop", s_remain, dist_to_goal
        );
        return;
    }

    double e_psi = wrap_angle(proj.heading - yaw);
    const double e_ct = proj.cross_track;

    // Path 2 records a reverse: geometric heading is ~π from the rover yaw.
    // Drive backward with a flipped heading error instead of wrapping ±π into a U-turn.
    bool reverse = false;
    if (std::abs(e_psi) > reverseAngle_) {
        reverse = true;
        e_psi = wrap_angle(e_psi - std::copysign(kPi, e_psi));
    }

    const bool arriving = s_remain < slowRadius_ || dist_to_goal < slowRadius_;

    // Three-case speed rule — no PID on speed (ArduRover closes that loop).
    double v = maxSpeed_;
    if (std::abs(e_psi) > pivotAngle_) {
        v = pivotSpeed_;
    }
    if (arriving) {
        const double arrive_r = std::min(s_remain, dist_to_goal);
        v = std::min(std::abs(v), maxSpeed_ * (arrive_r / slowRadius_));
    }
    v = clamp(v, 0.0, maxSpeed_);
    if (reverse) {
        // Keep a reverse crawl in the middle of the path, but do not force
        // pivotSpeed_ through the last waypoint — that drives past the goal.
        v = arriving ? -v : -std::max(v, pivotSpeed_);
    }

    const double vx = odom.twist.twist.linear.x;
    const double vy = odom.twist.twist.linear.y;
    const double speed_meas = std::hypot(vx, vy);
    const double v_denom = std::max(speed_meas + kSoft_, 1e-3);

    // Stanley: δ = e_ψ + arctan(k * e_ct / (v + k_soft)). Positive → turn left (FLU).
    const double delta = wrap_angle(e_psi + std::atan(kGain_ * e_ct / v_denom));
    double omega = clamp(delta, -maxYawRate_, maxYawRate_);
    if (std::abs(e_psi) > pivotAngle_) {
        omega = (e_psi >= 0.0 ? 1.0 : -1.0) * maxYawRate_;
    }

    if (arriving) {
        stuckTicks_ = 0;
    } else if (speed_meas < stuckSpeedEps_) {
        ++stuckTicks_;
    } else {
        stuckTicks_ = 0;
    }

    if (!arriving && stuckTicks_ >= stuckTicksLimit_ * 3) {
        const bool reverse_nudge = (stuckTicks_ / 10) % 2 == 0;
        v = reverse_nudge ? -0.35 : 0.2;
        omega = (e_psi >= 0.0 ? 1.0 : -1.0) * maxYawRate_;
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000, "Stuck recovery: rock %s (e_psi=%.2f)",
            reverse_nudge ? "reverse" : "forward", e_psi
        );
    } else if (!arriving && stuckTicks_ >= stuckTicksLimit_) {
        v = 0.0;
        omega = (e_psi >= 0.0 ? 1.0 : -1.0) * maxYawRate_;
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000, "Stuck recovery: pivoting in place (e_psi=%.2f)", e_psi
        );
    }

    PublishTwist(v, omega);

    RCLCPP_INFO_STREAM_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "v=" << v << " w=" << omega << " d=" << delta << " e_psi=" << e_psi << " e_ct=" << e_ct << " rev=" << reverse
             << " s=" << proj.s << "/" << refPath_.Length() << " dgoal=" << dist_to_goal
    );
}

}  // namespace ardurover_nav
