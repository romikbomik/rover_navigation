#pragma once

#include <cstddef>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "ardurover_nav/ardurover_controller.hpp"

namespace ardurover_nav {

// Pure-pursuit tracker. Owns path projection locally so the shared
// ReferencePath / launch / scorer stay unchanged.
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
        size_t seg_index{0};
        double t{0.0};
        double x{0.0};
        double y{0.0};
        double s{0.0};
        double dist{0.0};
    };

    void BuildTrack(const std::vector<Waypoint>& path);
    Projection ProjectOntoPath(double px, double py) const;
    PathPoint PointAtArcLength(double s) const;
    double PathCurvatureNear(double s) const;
    void PublishLookaheadMarker(double x, double y) const;

    std::vector<PathPoint> track_;
    double pathLength_{0.0};
    size_t lastSeg_{0};
    bool goalReached_{false};
    int stuckTicks_{0};

    double maxSpeed_{1.0};
    double lookaheadMin_{1.5};
    double lookaheadMax_{4.0};
    double lookaheadBase_{1.2};
    double lookaheadKv_{1.0};
    double maxYawRate_{1.0};
    double latAccelMax_{1.5};
    double decel_{0.8};
    double pivotAngle_{0.8};
    double pivotSpeed_{0.15};
    double goalTolerance_{0.25};
    double yawRateSign_{1.0};
    double omegaSpeedFloor_{0.35};
    double curvaturePreview_{2.0};
    double stuckSpeedEps_{0.08};
    int stuckTicksLimit_{20};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookaheadPub_;
};

}  // namespace ardurover_nav
