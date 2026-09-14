#pragma once

#include "ardurover_nav/ardurover_controller.hpp"
#include "ardurover_nav/reference_path.hpp"

namespace ardurover_nav {

// Geometric Stanley tracker. Steering = heading error + arctan(k * CTE / speed).
class ControllerStanley : public ArduroverController {
  public:
    ControllerStanley(rclcpp::Node& node, std::vector<Waypoint> path);

    void Control(const nav_msgs::msg::Odometry& odom) override;

  private:
    ReferencePath refPath_;
    bool goalReached_{false};
    int stuckTicks_{0};

    double kGain_{2.0};
    double kSoft_{1.0};
    double maxSpeed_{1.0};
    double maxYawRate_{1.0};
    double pivotAngle_{1.0};
    double pivotSpeed_{0.15};
    double reverseAngle_{2.0};
    double slowRadius_{1.5};
    double goalTolerance_{0.25};
    double stuckSpeedEps_{0.08};
    int stuckTicksLimit_{20};
};

}  // namespace ardurover_nav
