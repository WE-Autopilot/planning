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
#include <string>

#include "std_msgs/msg/string.hpp"
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
PlannerNode::PlannerNode(double rate_hz, std::string transitions_path): Node("planner_node"), rate_hz_(rate_hz), fsm(this->now(), fsm::VehicleState::Driving, transitions_path), event_generator()
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
    this->odometer_sub_ = create_subscription<FloatStamped>(
        "/ap1/mapping/odometer", 1,
        std::bind(&PlannerNode::on_odometer, this, std::placeholders::_1)
    );
    this->entities_sub_ = create_subscription<EntityStateArray>(
        "/ap1/mapping/entities", 1,
        std::bind(&PlannerNode::on_entities, this, std::placeholders::_1)
    );
    this->speed_sub_ = create_subscription<FloatStamped>(
        "/ap1/actuation/speed", 1,
        std::bind(&PlannerNode::on_speed, this, std::placeholders::_1)
    );

    // # Publishers
    this->state_pub_ = create_publisher<std_msgs::msg::String>("/ap1/planning/state", 1);
    this->target_path_pub_ = create_publisher<TargetPathStamped>("/ap1/planning/target_path", 1);
    this->speed_profile_pub_ = create_publisher<SpeedProfileStamped>("/ap1/planning/speed_profile", 1);

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
    // check that we have all the necessary fields
    if (this->odometer_ == nullptr || this->current_lane_ == nullptr || this->entities_ == nullptr) { // TODO: add age check here too
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "1 or more necessary field is null. Skipping loop.\nOd:%p\nLane:%p\nEntities:%p",
            (void *)this->odometer_.get(), (void *)this->current_lane_.get(), (void *)this->entities_.get()
        );
        return;
    }

    // assemble frame
    const frames::MapF map_f{
        this->speed,
        this->odometer_->value,
        this->now(),
        *this->current_lane_,
        *this->entities_
    };

    // process events
    auto events = this->event_generator.update(map_f, this->drive_through_start, this->stop_time, this->now());

    // handle state
    fsm::VehicleState previous_state = this->fsm.current_state;
    if (!events.empty()) {
        // update next state
        this->fsm.current_state = fsm::next_state(this->fsm.current_state, events, this->fsm.transitions);
    }
    if (this->fsm.current_state != previous_state) { // if the state has changed
        fsm::on_state_entry(previous_state, this->fsm.current_state, map_f, this->fsm.context);

        // publish state
        std::string s(fsm::to_string(this->fsm.current_state).value());
        std_msgs::msg::String msg;
        msg.data = s;
        this->state_pub_->publish(msg);
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

    // TODO: move this to init somehow - no need to publish every frame
    // Publish quick default state message for console
    std_msgs::msg::String msg;
    msg.data = fsm::to_string(this->fsm.current_state).value();
    this->state_pub_->publish(msg);
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

void PlannerNode::on_odometer(const FloatStamped::SharedPtr msg) {
    this->odometer_ = msg;
}

void PlannerNode::on_entities(const EntityStateArray::SharedPtr msg) {
    this->entities_ = msg;
}

} // namespace ap1::planning
