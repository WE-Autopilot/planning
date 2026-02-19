#ifndef AP1_PLANNING_NODE_HPP
#define AP1_PLANNING_NODE_HPP

#include <cmath>
#include <rclcpp/timer.hpp>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"

#include "ap1/planning/math_utils.hpp"

#define MAX_PLAN_AHEAD_WAYPOINTS 5

using ap1_msgs::msg::SpeedProfileStamped;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::FloatStamped;
using ap1_msgs::msg::LaneBoundaries;

using geometry_msgs::msg::Point;
using rclcpp::TimerBase;

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
    float target_speed_ = 0;
    const double rate_hz_;

    // State
    

    // Memory
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
     * @brief Calculates the centerline from the current lane boundaries.
     * @param lane The lane containing left and right boundaries.
     * @return std::vector<geometry_msgs::msg::Point> The calculated centerline.
     */
    std::vector<vec2f> calculate_centerline(const LaneBoundaries::SharedPtr lane);

    /**
     * @brief Planning loop callback runs rate_hz times per second.
     * This callback is responsible for sending commands & updates to control.
     * See P&C design for all Planning Loops.
     * See PCI for planning-control interface.
     */
    void planning_loop_callback();

    /**
     * @brief Creates a target path based on information known to the class - primarily map data.
     * Target paths are lists of waypoints, each relative to the car.
     *
     * Timestamp is set at time of **creation** not the time the route is sent. This is because TTL
     * checks on paths should always be measured to when it was created or else paths could be sent
     * later and TTL checks would pass on old paths -- i.e., not good.
     *
     * @return TargetPathStamped
     */
    TargetPathStamped create_route();

    /**
     * @brief Create a speed profile object.
     * Speed profiles are lists of speeds that control should follow. They map to path nodes and
     * apply to all subsequent nodes.
     *
     * For example,
     * A speed profile of { 1.0 } applies a 1m/s speed target to all waypoints in the target path.
     * A speed profile of { 1.0, 2.0 } applies a 1m/s speed target to the first waypoint and 2m/s
     * target to all following waypoints.
     *
     * @return SpeedProfileStamped
     */
    SpeedProfileStamped create_speed_profile();
};
} // namespace ap1::planning

#endif // AP1_PLANNING_NODE_HPP
