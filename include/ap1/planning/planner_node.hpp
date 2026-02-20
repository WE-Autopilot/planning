#ifndef AP1_PLANNING_NODE_HPP
#define AP1_PLANNING_NODE_HPP

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "geometry_msgs/msg/point.hpp"


#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/event_generator.hpp"

#define MAX_PLAN_AHEAD_WAYPOINTS 5

using ap1_msgs::msg::FloatStamped;
using ap1_msgs::msg::LaneBoundaries;
using ap1_msgs::msg::EntityStateArray;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::SpeedProfileStamped;

using rclcpp::TimerBase;
using geometry_msgs::msg::Point;

namespace ap1::planning
{

/**
 * @brief Planner Node class. See latest PCI for interface documentation.
 */
class PlannerNode : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Planner Node object
     *
     * @param rate_hz Primary loop update frequency.
     */
    PlannerNode(double rate_hz = 60.0);

  private:
    const double rate_hz_;
    float target_speed_ = 0;

    fsm::FSM fsm;
    EventGenerator event_generator = EventGenerator();

    // Memory
    FloatStamped::SharedPtr odometer_;
    EntityStateArray::SharedPtr entities_;
    LaneBoundaries::SharedPtr current_lane_;

    // Timer
    TimerBase::SharedPtr timer_;

    // Subscriptions
    rclcpp::Subscription<LaneBoundaries>::SharedPtr lane_sub_; // mapping
    rclcpp::Subscription<Point>::SharedPtr target_location_sub_; // console
    rclcpp::Subscription<FloatStamped>::SharedPtr target_speed_sub_; // console

    // Publishers
    rclcpp::Publisher<SpeedProfileStamped>::SharedPtr speed_profile_pub_; // control
    rclcpp::Publisher<TargetPathStamped>::SharedPtr target_path_pub_; // control

    // # Callbacks
    void on_lanes(const LaneBoundaries::SharedPtr lanes);
    void on_target_location(const geometry_msgs::msg::Point::SharedPtr loc);
    void on_target_speed(const FloatStamped::SharedPtr speed);

    /**
     * @brief Planning loop callback runs rate_hz times per second.
     * This callback is responsible for sending commands & updates to control.
     * See P&C design for all Planning Loops.
     * See PCI for planning-control interface.
     */
    void planning_loop_callback();
};
} // namespace ap1::planning

#endif // AP1_PLANNING_NODE_HPP
