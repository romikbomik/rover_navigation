#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ardurover_nav/ardurover_controller.hpp"
#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

// Builds a path follower from a name: pure_pursuit (default), pid, or stanley.
std::unique_ptr<ArduroverController> MakeController(
    const std::string& algorithm, rclcpp::Node& node, std::vector<Waypoint> path
);

}  // namespace ardurover_nav
