#pragma once

#include "ardurover_nav/ardurover_controller.hpp"
#include "ardurover_nav/reference_path.hpp"

namespace ardurover_nav {

class ControllerPID : public ArduroverController {
  public:
    ControllerPID(rclcpp::Node& node, std::vector<Waypoint> path);

    void Control(const nav_msgs::msg::Odometry& odom) override;

  private:
    // Textbook scalar PID: u = kp*e + ki*integral + kd*derivative.
    double PidUpdate(double error, double dt);
    void ResetPid();

    ReferencePath refPath_;
    bool goalReached_{false};

    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double integralLimit_{0.5};
    double integral_{0.0};
    double prevError_{0.0};
    bool hasPrev_{false};

    double cteGain_{1.0};
    double maxSpeed_{1.0};
    double maxYawRate_{1.0};
    double pivotAngle_{1.0};
    double pivotSpeed_{0.15};
    double slowRadius_{1.5};
    double goalTolerance_{0.25};
};

}  // namespace ardurover_nav
