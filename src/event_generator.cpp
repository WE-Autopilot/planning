#include "ap1/planning/event_generator.hpp"

#include <iostream>
#include <vector>
#include <optional>

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/state_context.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "ap1/planning/frames.hpp"
#include "ap1/planning/planner_node.hpp"

#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"

using ap1::planning::fsm::Event;
using ap1::planning::frames::MapF;

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;

// Helpers
bool sign_is_close(const EntityStateArray& entities) {
    for (const EntityState& entity : entities.entities) {
        // skip signs behind us
        if (entity.x < 0) continue;

        // if the sign is close by
        if (entity.x < STOPPING_TRANSITION_DISTANCE) {
            return true;
        }
    }

    return false;
}

// Beef

/**
 * Create a new EventGenerator
 */
ap1::planning::EventGenerator::EventGenerator() {}


/**
 * Get the event generator to figure out all the events in the frame.
 * There are 4 events to check, see ap1::planning::fsm::Event for all options.
 *
 * TODO: This data flow is super clunky and ugly and should be fixed.
 */
std::vector<Event> ap1::planning::EventGenerator::update(
    const MapF& frame,
    fsm::StateContext &ctx,
    rclcpp::Time now
) {
    // output var
    std::vector<Event> events{};

    // do we see a sign?
    if (sign_is_close(frame.entities)) events.push_back(Event::SignDetected);

    // have we crossed enough distance to exit drive_through?
    if (ctx.drive_through_start_distance.has_value()) {
        if (frame.odometer - ctx.drive_through_start_distance.value() > DRIVE_THROUGH_DISTANCE) {
            events.push_back(Event::DriveThruDistanceCovered);
        }
    }

    // have we stopped?
    if (frame.speed <= 0.f + EPSILON) {
        events.push_back(Event::HasStopped);
    }

    // has enough time passed?
    if (ctx.stop_entry_time.has_value()) {
        rclcpp::Duration stop_duration = now - ctx.stop_entry_time.value();

        if (stop_duration > rclcpp::Duration::from_seconds(MIN_STOP_DURATION)) {
            events.push_back(Event::StopTimeElapsed);
        }
    }

    return events;
}
