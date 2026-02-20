/**
 * AP1's beautiful, beautiful planning node.
 * CAVEATS:
 * - Loads map ONLY ONCE at the beginning of the runtime.
 * - assumes left-right lane lines have same number of waypoints
 * - assumes left-right lane lines are directly accross from eachother.
 *
 * We should assemble a smooth curve out of waypoints and determine centerline based on that
 * instead.
 */

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/frames.hpp"
#include "ap1/planning/behaviours.hpp"

#include "ap1/planning/planner_node.hpp"
#include "ap1/planning/event_generator.hpp"

#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"

using geometry_msgs::msg::Point;

using ap1_msgs::msg::SpeedProfileStamped;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::LaneBoundaries;

namespace ap1::planning
{
PlannerNode::PlannerNode(double rate_hz) : Node("planner_node"), rate_hz_(rate_hz), fsm({}), event_generator(EventGenerator())
{
    // # Subscribe to all inputs
    // todo: paths should be loaded from config
    this->lane_sub_ = create_subscription<LaneBoundaries>(
        "/ap1/mapping/lanes", 1,
        std::bind(&PlannerNode::on_lanes, this, std::placeholders::_1));
    // this->target_location_sub_ = create_subscription<Point>(
    //    "/ap1/control/target_location", 1,
    //    std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));
    this->target_speed_sub_ = create_subscription<FloatStamped>(
        "/ap1/control/target_speed", 1,
        std::bind(&PlannerNode::on_target_speed, this, std::placeholders::_1));

    // # Publishers
    this->speed_profile_pub_ = create_publisher<SpeedProfileStamped>("/ap1/planning/speed_profile", 1);
    this->target_path_pub_ = create_publisher<TargetPathStamped>("/ap1/planning/target_path", 1);

    // # Create Planning Loop @ rate_hz
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0f / rate_hz),
        std::bind(&PlannerNode::planning_loop_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized.");
}

// # Methods

void PlannerNode::planning_loop_callback()
{
    // assemble frame
    const frames::MapF map_f{
        this->target_speed_,
        this->odometer_->value,
        *this->current_lane_,
        *this->entities_
    };

    // process events
    auto event = this->event_generator.update();
    if (event) {
        // update next state
        this->fsm.current_state = fsm::next_state(this->fsm.current_state, *event, this->fsm.transitions);
    }

    // plan the route
    frames::RouteF route_f = behaviors::run_behaviour(this->fsm.current_state, map_f);

    // unwrap to route and speed profile messages
    TargetPathStamped path;
    SpeedProfileStamped speed_profile;
    frames::unwrap_route_f(route_f, path, speed_profile);

    // publish
    target_path_pub_->publish(path);
    speed_profile_pub_->publish(speed_profile);
}

// # Callbacks
void PlannerNode::on_lanes(const LaneBoundaries::SharedPtr lane)
{
    this->current_lane_ = lane;
}

void PlannerNode::on_target_speed(const FloatStamped::SharedPtr msg)
{
    this->target_speed_ = msg->value;

    std::string s = "Command: set speed to " + std::to_string(this->target_speed_);
    RCLCPP_INFO_STREAM(this->get_logger(), s);
}

} // namespace ap1::planning
