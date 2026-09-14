#pragma once

#include "ardurover_nav/ardurover_controller.hpp"
#include "ardurover_nav/pid.hpp"
#include "ardurover_nav/reference_path.hpp"

namespace ardurover_nav {

class ControllerPID : public ArduroverController {
  public:
    ControllerPID(rclcpp::Node& node, std::vector<Waypoint> path);

    void Control(const nav_msgs::msg::Odometry& odom) override;

  private:
    ReferencePath refPath_;
    Pid pid_;
    bool goalReached_{false};

    double cteGain_{1.0};
    double maxSpeed_{1.0};
    double maxYawRate_{1.0};
    double pivotAngle_{1.0};
    double pivotSpeed_{0.15};
    double slowRadius_{1.5};
    double goalTolerance_{0.25};
};

}  // namespace ardurover_nav
