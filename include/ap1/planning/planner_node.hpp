/**
 * Created: Oct. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_PLANNING_NODE_HPP
#define AP1_PLANNING_NODE_HPP

#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

namespace ap1::planning
{
class PlannerNode : public rclcpp::Node
{
  public:
    PlannerNode(double rate_hz = 60.0);

  private:
    // # Callbacks
    void on_hd_map(const std_msgs::msg::Float32MultiArray::SharedPtr); // WRONG TYPE SHOULD BE XML

    void on_turn_angle(const ap1_msgs::msg::TurnAngleStamped::SharedPtr);

    void on_vehicle_speed(const ap1_msgs::msg::VehicleSpeedStamped::SharedPtr);

    void on_target_location(const geometry_msgs::msg::Point::SharedPtr loc);

    void on_target_speed(const ap1_msgs::msg::VehicleSpeedStamped::SharedPtr);

    void planning_loop_callback();

    ap1_msgs::msg::TargetPathStamped create_route();

    ap1_msgs::msg::SpeedProfileStamped create_speed_profile();

    // # Subscriptions
    // Note: use SharedPtrs for all messages since they're dynamically allocated
    // in ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
        hd_map_sub_; // WRONG TYPE SHOULD BE XML
    rclcpp::Subscription<ap1_msgs::msg::TurnAngleStamped>::SharedPtr
        turn_angle_sub_; // the car's current turn angle, may not be needed
    rclcpp::Subscription<ap1_msgs::msg::VehicleSpeedStamped>::SharedPtr vehicle_speed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_location_sub_;
    rclcpp::Subscription<ap1_msgs::msg::VehicleSpeedStamped>::SharedPtr target_speed_sub_;

    // # Publishers
    rclcpp::Publisher<ap1_msgs::msg::SpeedProfileStamped>::SharedPtr speed_profile_pub_;
    rclcpp::Publisher<ap1_msgs::msg::TargetPathStamped>::SharedPtr target_path_pub_;

    // # Primary loop
    const double rate_hz_;
    float speed_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace ap1::planning

#endif // AP1_PLANNING_NODE_HPP
