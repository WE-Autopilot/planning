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

namespace ap1::planning {
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode(double rate_hz = 60.0);

private:
  // # Callbacks
  void on_hd_map(const std_msgs::msg::Float32MultiArray::SharedPtr);

  void on_turn_angle(const std_msgs::msg::Float32::SharedPtr);

  void on_vehicle_speed(const std_msgs::msg::Float32::SharedPtr);

  void on_target_location(const geometry_msgs::msg::Point::SharedPtr msg);

  void planning_loop_callback();

  // # Subscriptions
  // Note: use SharedPtrs for all messages since they're dynamically allocated
  // in ROS
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hd_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr turn_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vehicle_speed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
      target_location_sub_;

  // # Publishers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      speed_profile_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      target_path_pub_;

  // # Primary loop
  const double rate_hz_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace ap1::planning

#endif // AP1_PLANNING_NODE_HPP
