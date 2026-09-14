#include "ardurover_nav/controller_factory.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include "ardurover_nav/controller_impl/controller_pid.hpp"
#include "ardurover_nav/controller_impl/controller_pure_pursuit.hpp"
#include "ardurover_nav/controller_impl/controller_stanley.hpp"

namespace ardurover_nav {
namespace {

std::string NormalizeName(std::string name) {
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::replace(name.begin(), name.end(), '-', '_');
    return name;
}

}  // namespace

std::unique_ptr<ArduroverController> MakeController(
    const std::string& algorithm, rclcpp::Node& node, std::vector<Waypoint> path
) {
    const std::string name = NormalizeName(algorithm);

    if (name.empty() || name == "pure_pursuit") {
        return std::make_unique<ControllerPurePursuit>(node, std::move(path));
    }
    if (name == "pid") {
        return std::make_unique<ControllerPID>(node, std::move(path));
    }
    if (name == "stanley") {
        return std::make_unique<ControllerStanley>(node, std::move(path));
    }

    throw std::invalid_argument(
        "Unknown controller '" + algorithm + "'. Use pid, stanley, or pure_pursuit."
    );
}

}  // namespace ardurover_nav
