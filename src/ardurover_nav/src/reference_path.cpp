#include "ardurover_nav/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ardurover_nav {
namespace {

double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

}  // namespace

ReferencePath::ReferencePath(std::vector<Waypoint> waypoints) {
    constexpr double kMinSpacing = 0.05;
    points_.clear();
    for (const auto& wp : waypoints) {
        if (!points_.empty()) {
            const double dx = wp.x - points_.back().x;
            const double dy = wp.y - points_.back().y;
            if (dx * dx + dy * dy < kMinSpacing * kMinSpacing) {
                continue;
            }
        }
        Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.s = points_.empty() ? 0.0
                              : points_.back().s + std::hypot(wp.x - points_.back().x, wp.y - points_.back().y);
        points_.push_back(p);
    }
    if (points_.empty()) {
        throw std::runtime_error("ReferencePath is empty after de-duplication");
    }
    length_ = points_.back().s;
}

PathProjection ReferencePath::Project(double px, double py) {
    PathProjection best;
    best.cross_track = std::numeric_limits<double>::infinity();

    if (points_.size() < 2) {
        best.x = points_.front().x;
        best.y = points_.front().y;
        best.s = 0.0;
        best.heading = 0.0;
        best.cross_track = std::hypot(px - best.x, py - best.y);
        return best;
    }

    constexpr double kLookBack = 2.0;
    constexpr double kLookAhead = 10.0;
    const double s_ref = points_[std::min(last_seg_, points_.size() - 1)].s;
    size_t i_start = last_seg_;
    size_t i_end = last_seg_;
    while (i_start > 0 && points_[i_start].s > s_ref - kLookBack) {
        --i_start;
    }
    while (i_end + 1 < points_.size() && points_[i_end].s < s_ref + kLookAhead) {
        ++i_end;
    }
    const size_t last_seg_idx = points_.size() - 2;
    const size_t seg_end = std::min(i_end, last_seg_idx);

    double best_abs = std::numeric_limits<double>::infinity();

    auto consider = [&](size_t i) {
        const double ax = points_[i].x;
        const double ay = points_[i].y;
        const double bx = points_[i + 1].x;
        const double by = points_[i + 1].y;
        const double abx = bx - ax;
        const double aby = by - ay;
        const double length2 = abx * abx + aby * aby;
        double t = 0.0;
        if (length2 > 1e-12) {
            t = clamp(((px - ax) * abx + (py - ay) * aby) / length2, 0.0, 1.0);
        }
        const double cx = ax + t * abx;
        const double cy = ay + t * aby;
        const double dist = std::hypot(px - cx, py - cy);
        if (dist < best_abs) {
            best_abs = dist;
            const double heading = std::atan2(aby, abx);
            // Signed cross-track: positive when path is to the left of the rover.
            // (p* - p_rover) · n with n = (-sin ψ, cos ψ)
            const double nx = -std::sin(heading);
            const double ny = std::cos(heading);
            best.cross_track = (cx - px) * nx + (cy - py) * ny;
            best.x = cx;
            best.y = cy;
            best.s = points_[i].s + t * (points_[i + 1].s - points_[i].s);
            best.heading = heading;
            best.seg_index = i;
        }
    };

    for (size_t i = i_start; i <= seg_end; ++i) {
        consider(i);
    }
    if (!std::isfinite(best_abs)) {
        for (size_t i = 0; i <= last_seg_idx; ++i) {
            consider(i);
        }
    }

    last_seg_ = best.seg_index;
    return best;
}

double ReferencePath::HeadingAt(double s) const {
    if (points_.size() < 2) {
        return 0.0;
    }
    s = clamp(s, 0.0, length_);
    size_t i = 0;
    while (i + 1 < points_.size() && points_[i + 1].s < s) {
        ++i;
    }
    if (i + 1 >= points_.size()) {
        i = points_.size() - 2;
    }
    return std::atan2(points_[i + 1].y - points_[i].y, points_[i + 1].x - points_[i].x);
}

double ReferencePath::RemainingLength(double s) const {
    return std::max(0.0, length_ - clamp(s, 0.0, length_));
}

}  // namespace ardurover_nav
