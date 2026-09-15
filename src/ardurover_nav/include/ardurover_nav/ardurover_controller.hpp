#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

// Abstract base: owns MAVROS bring-up and command publishing.
// Derived classes implement Control().
class ArduroverController {
  public:
    ArduroverController(rclcpp::Node& node, std::vector<Waypoint> path);
    virtual ~ArduroverController() = default;

    bool SetupArdurover();
    virtual void Control(const nav_msgs::msg::Odometry& odom) = 0;

  protected:
    void PublishTwist(double linear_x, double angular_z);
    void PublishStop();
    double TickDt(const rclcpp::Time& stamp);
    static double WrapAngle(double a);

    rclcpp::Node& node_;
    std::vector<Waypoint> path_;

  private:
    enum class SetupState { WaitServices, SetFrame, Prime, ArmAndGuide, Ready };

    void RequestGuided();
    void RequestArm();

    SetupState setupState_{SetupState::WaitServices};
    int primeTicks_{0};
    int setupTicks_{0};
    mavros_msgs::msg::State::SharedPtr mavState_;
    std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> modeFuture_;
    std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> armFuture_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setMode_;
    rclcpp::AsyncParametersClient::SharedPtr paramClient_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub_;
    std::optional<rclcpp::Time> prevStamp_;
};

}  // namespace ardurover_nav
