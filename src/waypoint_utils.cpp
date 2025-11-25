#include "ap1/planning/waypoint_utils.hpp"
#include <limits>

namespace ap1::planning {

size_t locate_closest_waypoint(const vec2f& target_location,
                               const std::vector<vec2f>& navigable_waypoints)
{
    // Check if navigable_waypoints is empty
    if (navigable_waypoints.empty()) {
        return static_cast<size_t>(-1);
    }

    // Initialize with the first waypoint
    float min_distance = distance(target_location, navigable_waypoints[0]);
    size_t closest_index = 0;

    // Loop through all waypoints to find the closest one
    for (size_t i = 1; i < navigable_waypoints.size(); ++i) {
        float current_distance = distance(target_location, navigable_waypoints[i]);
        if (current_distance < min_distance) {
            min_distance = current_distance;
            closest_index = i;
        }
    }

    return closest_index;
}

std::vector<vec2f> generate_waypoint_sequence(const std::vector<vec2f>& waypoints,
                                              const size_t to,
                                              const std::vector<vec2f>& fallback_path)
{
    // Check if waypoints is empty OR to == -1 (no valid target)
    if (waypoints.empty() || to == static_cast<size_t>(-1)) {
        return fallback_path;
    }

    // Create a new vector to hold the sequence
    std::vector<vec2f> sequence;

    // Copy waypoints from index 0 to index 'to' (inclusive)
    // This creates the path from the car's position to the target
    for (size_t i = 0; i <= to && i < waypoints.size(); ++i) {
        sequence.push_back(waypoints[i]);
    }

    return sequence;
}

} // namespace ap1::planning
