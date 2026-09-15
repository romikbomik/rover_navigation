#pragma once

#include <cstddef>
#include <vector>

#include "ardurover_nav/ardurover_controller.hpp"

namespace ardurover_nav {

// Geometric pure-pursuit: chase waypoint i + lookaheadIndex_ on the input path.
class ControllerPurePursuit : public ArduroverController {
  public:
    ControllerPurePursuit(rclcpp::Node& node, std::vector<Waypoint> path);

    void Control(const nav_msgs::msg::Odometry& odom) override;

  private:
    size_t ClosestIndex(double px, double py) const;

    size_t lastIndex_{0};
    bool goalReached_{false};

    int lookaheadIndex_{2};
    double maxSpeed_{1.0};
    double maxYawRate_{1.0};
    double goalTolerance_{0.25};
    double yawRateSign_{1.0};
};

}  // namespace ardurover_nav
