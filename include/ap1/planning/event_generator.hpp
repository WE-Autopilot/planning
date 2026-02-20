/**
 * Event generator is responsible for determining events that occur in the world through mapping frames.
 * These are then consumed
 */

#ifndef AP1_PLANNING_EVENT_GEN_HPP
#define AP1_PLANNING_EVENT_GEN_HPP

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/frames.hpp"
#include "rclcpp/time.hpp"
#include <optional>

#define MIN_STOP_DURATION 3 // s
#define DRIVE_THROUGH_DISTANCE 1 // m
#define STOPPING_TRANSITION_DISTANCE 3 // m

using ap1::planning::frames::MapF;

namespace ap1::planning {
class EventGenerator {
public:
    EventGenerator();

    std::vector<ap1::planning::fsm::Event> update(
        const MapF& map,
        std::optional<float> drive_through_start,
        std::optional<rclcpp::Time> stop_time,
        rclcpp::Time now
    );    
};
}

#endif // AP1_PLANNING_EVENT_GEN_HPP