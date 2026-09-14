#include "ardurover_nav/ControllerImpl/controller_pid.hpp"

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

double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

}  // namespace

ControllerPID::Pid::Pid(double kp, double ki, double kd, double integral_limit)
    : kp_(kp), ki_(ki), kd_(kd), integralLimit_(integral_limit) {}

double ControllerPID::Pid::Update(double error, double dt) {
    if (dt <= 0.0) {
        return kp_ * error;
    }

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integralLimit_, integralLimit_);

    const double derivative = hasPrev_ ? (error - prevError_) / dt : 0.0;
    prevError_ = error;
    hasPrev_ = true;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void ControllerPID::Pid::Reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    hasPrev_ = false;
}

void ControllerPID::Pid::SetGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void ControllerPID::Pid::SetIntegralLimit(double limit) {
    integralLimit_ = limit;
}

ControllerPID::ControllerPID(rclcpp::Node& node, std::vector<Waypoint> path)
    : ArduroverController(node, path), refPath_(path_), pid_(0.0, 0.0, 0.0, 0.5) {
    const double kp = node.declare_parameter("pid_kp", 1.5);
    const double ki = node.declare_parameter("pid_ki", 0.0);
    const double kd = node.declare_parameter("pid_kd", 0.2);
    const double i_lim = node.declare_parameter("pid_integral_limit", 0.5);
    pid_.SetGains(kp, ki, kd);
    pid_.SetIntegralLimit(i_lim);

    cteGain_ = node.declare_parameter("cte_gain", 1.0);
    maxSpeed_ = node.declare_parameter("max_speed", 1.0);
    maxYawRate_ = node.declare_parameter("max_yaw_rate", 1.0);
    pivotAngle_ = node.declare_parameter("pivot_angle", 1.0);
    pivotSpeed_ = node.declare_parameter("pivot_speed", 0.15);
    slowRadius_ = node.declare_parameter("slow_radius", 1.5);
    goalTolerance_ = node.declare_parameter("goal_tolerance", 0.25);

    RCLCPP_INFO_STREAM(
        node_.get_logger(), "ControllerPID ready: path length " << refPath_.Length() << " m, kp/ki/kd = " << kp << "/"
                                                                << ki << "/" << kd
    );
}

void ControllerPID::Control(const nav_msgs::msg::Odometry& odom) {
    if (goalReached_) {
        PublishStop();
        return;
    }

    const double dt = TickDt(odom.header.stamp);
    const double px = odom.pose.pose.position.x;
    const double py = odom.pose.pose.position.y;
    const double yaw = yaw_from_quat(odom.pose.pose.orientation);

    const PathProjection proj = refPath_.Project(px, py);
    const double s_remain = refPath_.RemainingLength(proj.s);

    if (s_remain < goalTolerance_) {
        goalReached_ = true;
        pid_.Reset();
        PublishStop();
        RCLCPP_INFO(node_.get_logger(), "Goal reached (s_remain=%.2f m) — holding stop", s_remain);
        return;
    }

    const double e_psi = wrap_angle(proj.heading - yaw);
    const double e_ct = proj.cross_track;
    const double e = e_psi + cteGain_ * e_ct;

    double omega = pid_.Update(e, dt);
    omega = clamp(omega, -maxYawRate_, maxYawRate_);

    // Three-case speed rule — no PID on speed (ArduRover closes that loop).
    double v = maxSpeed_;
    if (std::abs(e_psi) > pivotAngle_) {
        v = pivotSpeed_;
    }
    if (s_remain < slowRadius_) {
        v = std::min(v, maxSpeed_ * (s_remain / slowRadius_));
    }
    v = clamp(v, 0.0, maxSpeed_);

    PublishTwist(v, omega);

    RCLCPP_INFO_STREAM_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "v=" << v << " w=" << omega << " e=" << e << " e_psi=" << e_psi << " e_ct=" << e_ct << " s=" << proj.s << "/"
             << refPath_.Length()
    );
}

}  // namespace ardurover_nav
