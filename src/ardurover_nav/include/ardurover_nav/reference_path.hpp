#pragma once

#include <cstddef>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

struct PathProjection {
    double x{0.0};
    double y{0.0};
    double s{0.0};
    double heading{0.0};
    double cross_track{0.0};  // signed: positive when path is to the left of the rover
    size_t seg_index{0};
};

// De-duplicated polyline with arc-length indexing and windowed projection.
class ReferencePath {
  public:
    explicit ReferencePath(std::vector<Waypoint> waypoints);

    PathProjection Project(double x, double y);
    double HeadingAt(double s) const;
    double RemainingLength(double s) const;
    double Length() const { return length_; }
    bool Empty() const { return points_.empty(); }

  private:
    struct Point {
        double x{0.0};
        double y{0.0};
        double s{0.0};
    };

    std::vector<Point> points_;
    double length_{0.0};
    size_t last_seg_{0};
};

}  // namespace ardurover_nav
