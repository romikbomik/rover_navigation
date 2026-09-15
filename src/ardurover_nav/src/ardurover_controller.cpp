#include "ardurover_nav/ardurover_controller.hpp"

#include <chrono>
#include <memory>

namespace ardurover_nav {

ArduroverController::ArduroverController(rclcpp::Node& node, std::vector<Waypoint> path)
    : node_(node), path_(std::move(path)) {
    arming_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    setMode_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, "/mavros/setpoint_velocity");
    cmdPub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
    );
    stateSub_ = node_.create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, [this](mavros_msgs::msg::State::SharedPtr msg) { mavState_ = std::move(msg); }
    );
}

void ArduroverController::PublishTwist(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmdPub_->publish(cmd);
}

void ArduroverController::PublishStop() {
    PublishTwist(0.0, 0.0);
}

double ArduroverController::TickDt(const rclcpp::Time& stamp) {
    double dt = 1.0 / 20.0;  // nominal control rate
    if (prevStamp_) {
        dt = (stamp - *prevStamp_).seconds();
        if (dt <= 1e-4 || dt > 0.5) {
            dt = 1.0 / 20.0;
        }
    }
    prevStamp_ = stamp;
    return dt;
}

void ArduroverController::RequestGuided() {
    if (modeFuture_.valid() && modeFuture_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "GUIDED";
    modeFuture_ = setMode_->async_send_request(req).future.share();
}

void ArduroverController::RequestArm() {
    if (armFuture_.valid() && armFuture_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    armFuture_ = arming_->async_send_request(req).future.share();
}

bool ArduroverController::SetupArdurover() {
    switch (setupState_) {
        case SetupState::WaitServices:
            if (arming_->service_is_ready() && setMode_->service_is_ready()) {
                setupState_ = SetupState::SetFrame;
            }
            return false;
        case SetupState::SetFrame:
            if (!paramClient_->service_is_ready()) {
                return false;
            }
            paramClient_->set_parameters({rclcpp::Parameter("mav_frame", "BODY_NED")});
            setupState_ = SetupState::Prime;
            return false;
        case SetupState::Prime:
            PublishStop();
            if (++primeTicks_ >= 20) {
                setupState_ = SetupState::ArmAndGuide;
                setupTicks_ = 0;
            }
            return false;
        case SetupState::ArmAndGuide: {
            // Stream zeros and retry until FCU is fully up; early SetMode is otherwise ignored.
            PublishStop();
            ++setupTicks_;

            const bool connected = mavState_ && mavState_->connected;
            const bool armed = mavState_ && mavState_->armed;
            const bool guided = mavState_ && mavState_->mode == "GUIDED";
            if (!connected) {
                return false;
            }
            if (!armed && (setupTicks_ % 20 == 1)) {
                RequestArm();
                RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "Requesting arm...");
            }
            // Retry GUIDED every second — early requests are often ignored until the FCU is ready.
            if (!guided && (setupTicks_ % 20 == 1 || setupTicks_ % 20 == 10)) {
                RequestGuided();
                RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "Requesting GUIDED...");
            }
            if (armed && guided) {
                setupState_ = SetupState::Ready;
                RCLCPP_INFO(node_.get_logger(), "Controller running");
                return true;
            }
            return false;
        }
        case SetupState::Ready:
            if (mavState_ && mavState_->mode != "GUIDED") {
                RequestGuided();
            }
            return true;
    }
    return false;
}

}  // namespace ardurover_nav
