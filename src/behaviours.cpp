#include "ap1/planning/behaviours.hpp"

#include <algorithm>
#include <stdexcept>

#include "ap1/planning/frames.hpp"
#include "ap1/planning/math_utils.hpp"
#include "ap1/planning/planner_node.hpp"
#include "ap1/planning/waypoint_utils.hpp"

#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"

using namespace ap1::planning;
using namespace ap1::planning::behaviors;

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;

// Helpers
bool sign_is_close(const EntityStateArray &entities);
const EntityState* get_next_sign(const EntityStateArray& entities);
std::vector<vec2f> calculate_centerline(const ap1_msgs::msg::LaneBoundaries &lane);

// Behavior Handlers

/**
 * Handle driving.
 * The current behavior is centerline following with constant speed.
 */
frames::RouteF handle_driving(const frames::MapF &map) {
    // Check if there is a usable lane in map
    bool lane_is_usable = !map.lane.left.empty() && !map.lane.right.empty();
    if (!lane_is_usable) {
        return frames::RouteF{{}, {}}; // return empty
    }

    // If there is a usable lane, calculate its centerline
    std::vector<vec2f> centerline = calculate_centerline(map.lane); // throws on failure btw

    // Find the starting waypoint on this centerline
    long start_idx = find_next_waypoint_idx(centerline); // first waypoint ahead of car in lane
    if (start_idx == -1) {
        // if we failed to find one throw an error
        throw std::runtime_error("Failed to find upcoming waypoint in lane centerline!");
    }
    auto start = centerline.begin() + std::min(static_cast<unsigned long>(start_idx), centerline.size());

    // Find the ending waypoint on this centerline
    auto end = centerline.begin() + std::min(static_cast<unsigned long>(start_idx) + MAX_PLAN_AHEAD_WAYPOINTS, centerline.size()); // replace with navigation later

    // Slice the centerline vector to get the route
    std::vector<vec2f> route(start, end);

    // return the route
    return frames::RouteF{
        route,
        {TARGET_SPEED} // should be received from planning somehow
    };
}

/**
 * Handle stopping.
 * Current behavior is 1 waypoint at the stop sign with speed = 0.
 */
frames::RouteF handle_stopping(const frames::MapF &map) {
    const EntityState *next_sign = get_next_sign(map.entities);
    if (next_sign == nullptr) {
        throw std::runtime_error("Stopping with no upcoming sign?");
    }

    return {
        {vec2f{next_sign->x, next_sign->y}},
        {0}
    };
}

/**
 * Handle being stopped.
 * Current behavior is to sit still with v = 0.
 */
frames::RouteF handle_stopped(const frames::MapF &map) {
    return {
        {vec2f{0, 0}},
        {0}
    };
}

// Helpers
// ASSUMES ALL ENTITIES ARE STOP SIGNS
bool sign_is_close(const EntityStateArray &entities) {
    return false;
}

// ASSUMES ALL ENTITIES ARE STOP SIGNS
const EntityState* get_next_sign(const EntityStateArray& entities) {
    return nullptr;
}

std::vector<vec2f> calculate_centerline(const ap1_msgs::msg::LaneBoundaries &lane) {
    return {};
}
