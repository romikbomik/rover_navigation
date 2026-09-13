#pragma once

namespace ardurover_nav {

// Textbook scalar PID: u = kp*e + ki*integral + kd*derivative.
class Pid {
  public:
    Pid(double kp, double ki, double kd, double integral_limit);

    double Update(double error, double dt);
    void Reset();

    void SetGains(double kp, double ki, double kd);
    void SetIntegralLimit(double limit);

  private:
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double integralLimit_{0.0};
    double integral_{0.0};
    double prevError_{0.0};
    bool hasPrev_{false};
};

}  // namespace ardurover_nav
