#include "ardurover_nav/controller_impl/controller_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ardurover_nav {

ControllerPurePursuit::ControllerPurePursuit(rclcpp::Node& node, std::vector<Waypoint> path)
    : ArduroverController(node, path) {
    lookaheadIndex_ = static_cast<int>(node.declare_parameter("lookahead_index", 3));
    maxSpeed_ = node.declare_parameter("max_speed", 1.0);
    maxYawRate_ = node.declare_parameter("max_yaw_rate", 1.0);
    goalTolerance_ = node.declare_parameter("goal_tolerance", 0.25);
    yawRateSign_ = node.declare_parameter("yaw_rate_sign", 1.0);

    if (path_.empty()) {
        throw std::runtime_error("Path is empty");
    }
    lookaheadIndex_ = std::max(lookaheadIndex_, 1);

    RCLCPP_INFO_STREAM(
        node_.get_logger(), "ControllerPurePursuit ready: " << path_.size() << " waypoints, lookahead "
                                                            << lookaheadIndex_ << " indices"
    );
}

size_t ControllerPurePursuit::ClosestIndex(double px, double py) const {
    // Do not jump backward along the recording; a short look-back covers noise.
    const size_t start = (lastIndex_ > 0) ? lastIndex_ - 1 : 0;
    size_t best = start;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = start; i < path_.size(); ++i) {
        const double dist = std::hypot(path_[i].x - px, path_[i].y - py);
        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }
    return best;
}

void ControllerPurePursuit::Control(const nav_msgs::msg::Odometry& odom) {
    if (goalReached_) {
        PublishStop();
        return;
    }

    const double px = odom.pose.pose.position.x;
    const double py = odom.pose.pose.position.y;
    const double yaw = yaw_from_quat(odom.pose.pose.orientation);

    const size_t i = ClosestIndex(px, py);
    lastIndex_ = i;

    const Waypoint& goal = path_.back();
    const double dist_to_goal = std::hypot(px - goal.x, py - goal.y);
    // Closest index, not lookahead: i + lookaheadIndex_ is still several samples short.
    if (i >= path_.size() - 1 && dist_to_goal < goalTolerance_) {
        goalReached_ = true;
        RCLCPP_INFO(node_.get_logger(), "Goal reached (dist=%.2f m) — holding stop", dist_to_goal);
        PublishStop();
        return;
    }

    const size_t target_i = std::min(i + static_cast<size_t>(lookaheadIndex_), path_.size() - 1);
    const Waypoint& target = path_[target_i];

    const double dx = target.x - px;
    const double dy = target.y - py;
    const double ld = std::hypot(dx, dy);
    const double bearing = std::atan2(dy, dx);
    const double alpha = WrapAngle(bearing - yaw);

    // κ = 2 sin(α) / Ld, with Ld the chord to the chosen waypoint.
    const double kappa = (2.0 * std::sin(alpha)) / std::max(ld, 1e-3);
    double v_cmd = maxSpeed_;
    if (dist_to_goal < 1.5) {
        v_cmd = std::min(v_cmd, maxSpeed_ * (dist_to_goal / 1.5));
        v_cmd = std::max(v_cmd, 0.15);
    }
    double omega = std::clamp(v_cmd * kappa, -maxYawRate_, maxYawRate_);
    omega *= yawRateSign_;

    PublishTwist(v_cmd, omega);

    RCLCPP_INFO_STREAM_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000, "v=" << v_cmd << " w=" << omega << " alpha=" << alpha
                                                           << " i=" << i << " tgt=" << target_i << " Ld=" << ld
                                                           << " cte=" << std::hypot(path_[i].x - px, path_[i].y - py)
    );
}

}  // namespace ardurover_nav
