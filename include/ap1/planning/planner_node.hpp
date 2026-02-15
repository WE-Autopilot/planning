/**
 * Created: Oct. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_PLANNING_NODE_HPP
#define AP1_PLANNING_NODE_HPP

#include <cmath>
#include <functional>
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"

#include "ap1/planning/math_utils.hpp"
#include "ap1/planning/waypoint_utils.hpp"

using ap1_msgs::msg::SpeedProfileStamped;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::FloatStamped;
using ap1_msgs::msg::LaneBoundaries;

using geometry_msgs::msg::Point;
using rclcpp::TimerBase;
using std_msgs::msg::Float32MultiArray;

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

    struct Lane
    {
        std::vector<geometry_msgs::msg::Point> left_boundary;
        std::vector<geometry_msgs::msg::Point> right_boundary;
    };

  private:
    float speed_ = 0;
    const double rate_hz_;
    Point target_location_;
    Lane current_lane_;
    std::string map_file_path_;
    lanelet::LaneletMapPtr lanelet_map_;

    TimerBase::SharedPtr timer_;

    // Subscriptions
    // Note: use SharedPtrs for all messages since they're dynamically allocated in ROS
    // All these types need to be updated twin.
    rclcpp::Subscription<LaneBoundaries>::SharedPtr hd_map_sub_;
    rclcpp::Subscription<FloatStamped>::SharedPtr vehicle_speed_sub_;
    rclcpp::Subscription<Point>::SharedPtr target_location_sub_;
    rclcpp::Subscription<FloatStamped>::SharedPtr target_speed_sub_;

    // Publishers
    rclcpp::Publisher<SpeedProfileStamped>::SharedPtr speed_profile_pub_;
    rclcpp::Publisher<TargetPathStamped>::SharedPtr target_path_pub_;

    // # Callbacks
    void on_lanes(const LaneBoundaries::SharedPtr);
    void on_turn_angle(const FloatStamped::SharedPtr);
    void on_vehicle_speed(const FloatStamped::SharedPtr);
    void on_target_location(const geometry_msgs::msg::Point::SharedPtr loc);
    void on_target_speed(const FloatStamped::SharedPtr);

    /**
     * @brief Mocks map data for testing.
     */
    void process_map_data();

    /**
     * @brief Calculates the centerline from the current lane boundaries.
     * @param lane The lane containing left and right boundaries.
     * @return std::vector<geometry_msgs::msg::Point> The calculated centerline.
     */
    std::vector<vec2f> calculate_centerline(const Lane& lane);

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
