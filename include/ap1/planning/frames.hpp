/**
 * Definition for the Route and Map frames.
 * See documentation on frames for more details.
 */

#ifndef AP1_PLANNING_FRAMES_HPP
#define AP1_PLANNING_FRAMES_HPP

#include <vector>

#include "ap1/planning/math_utils.hpp"

#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"

namespace ap1::planning::frames
{
struct RouteF
{
    std::vector<ap1::planning::vec2f> route;
    std::vector<float> speed_profile;
};

struct MapF
{
    float speed;    // m/s
    float odometer; // distance (m)
    ap1_msgs::msg::LaneBoundaries lane;
    ap1_msgs::msg::EntityStateArray entities;
};
} // namespace ap1::planning::frames

#endif // AP1_PLANNING_FRAMES_HPP