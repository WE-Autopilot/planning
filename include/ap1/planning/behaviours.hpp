/**
 * This contains callbacks for all of plannin's state behaviors.
 */
#ifndef AP1_PLANNING_BEHAVIORS
#define AP1_PLANNING_BEHAVIORS

#include "ap1/planning/frames.hpp"
#include "ap1/planning/fsm.hpp"

#define TARGET_SPEED 4.0f

using namespace ap1::planning;

namespace ap1::planning::behaviors {

// BehaviorFn type
using BehaviorFn = frames::RouteF(*)(const frames::MapF&); // consumes a map frame and produces a route frame

// Behavior Handlers
frames::RouteF handle_driving(const frames::MapF &map);
frames::RouteF handle_stopping(const frames::MapF &map);
frames::RouteF handle_stopped(const frames::MapF &map);

// Map
constexpr std::array<std::pair<fsm::VehicleState, BehaviorFn>, 6> behavior_table{{
    {fsm::VehicleState::Driving, handle_driving},
    {fsm::VehicleState::Stopping, handle_stopping},
    {fsm::VehicleState::Stopped, handle_stopped},
    {fsm::VehicleState::DrivingThrough, handle_driving} // same as driving
}};
}
#endif // AP1_PLANNING_BEHAVIOURS