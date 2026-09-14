#pragma once

#include <cstddef>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "ardurover_nav/ardurover_controller.hpp"

namespace ardurover_nav {

// Geometric pure-pursuit tracker. Steers toward a lookahead point on the path.
class ControllerPurePursuit : public ArduroverController {
  public:
    ControllerPurePursuit(rclcpp::Node& node, std::vector<Waypoint> path);

    void Control(const nav_msgs::msg::Odometry& odom) override;

  private:
    struct PathPoint {
        double x{0.0};
        double y{0.0};
        double s{0.0};
    };

    struct Projection {
        size_t segIndex{0};
        double t{0.0};
        double x{0.0};
        double y{0.0};
        double s{0.0};
        double dist{0.0};
    };

    void BuildTrack(const std::vector<Waypoint>& path);
    Projection ProjectOntoPath(double px, double py) const;
    void ConsiderSegment(Projection& best, size_t i, double px, double py) const;
    PathPoint PointAtArcLength(double s) const;
    PathPoint LookaheadPoint(double s0, double ld, double px, double py) const;
    double HeadingLimitedLookahead(double s, double ld_max, double max_dtheta) const;
    double RawHeadingAt(double s) const;
    double HeadingAt(double s) const;
    double PathCurvatureNear(double s, double preview) const;
    double WrapAngle(double a) const;
    void PublishLookaheadMarker(double x, double y) const;

    std::vector<PathPoint> track_;
    double pathLength_{0.0};
    size_t lastSeg_{0};
    bool goalReached_{false};
    int stuckTicks_{0};
    double omegaFilt_{0.0};
    bool haveOmegaFilt_{false};

    double maxSpeed_{1.0};
    double lookaheadMin_{0.75};
    double lookaheadMax_{2.5};
    double lookaheadBase_{0.6};
    double lookaheadKv_{1.0};
    double lookaheadKKappa_{0.4};
    double lookaheadMaxDTheta_{0.85};
    double maxYawRate_{1.0};
    double latAccelMax_{1.0};
    double decel_{0.8};
    double pivotAngle_{0.8};
    double pivotSpeed_{0.15};
    double reverseAngle_{2.0};
    double goalTolerance_{0.25};
    double yawRateSign_{1.0};
    double omegaSpeedFloor_{0.2};
    double omegaTau_{0.2};
    double curvaturePreview_{3.0};
    double stuckSpeedEps_{0.08};
    int stuckTicksLimit_{20};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookaheadPub_;
};

}  // namespace ardurover_nav
