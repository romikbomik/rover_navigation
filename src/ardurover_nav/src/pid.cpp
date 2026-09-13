#include "ardurover_nav/pid.hpp"

#include <algorithm>

namespace ardurover_nav {

Pid::Pid(double kp, double ki, double kd, double integral_limit)
    : kp_(kp), ki_(ki), kd_(kd), integralLimit_(integral_limit) {}

double Pid::Update(double error, double dt) {
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

void Pid::Reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    hasPrev_ = false;
}

void Pid::SetGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::SetIntegralLimit(double limit) {
    integralLimit_ = limit;
}

}  // namespace ardurover_nav
