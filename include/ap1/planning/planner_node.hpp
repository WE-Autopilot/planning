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
  PlannerNode(double rate_hz = 60.0) : Node("planner_node"), rate_hz_(rate_hz) {
    // # Subscribe to all inputs
    // - HD MAP
    hd_map_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "hd_map", 10,
        std::bind(&PlannerNode::on_hd_map, this, std::placeholders::_1));
    // - TURN ANGLE
    turn_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "turn_angle", 10,
        std::bind(&PlannerNode::on_turn_angle, this, std::placeholders::_1));
    // - VEHICLE SPEED
    vehicle_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "vehicle_speed", 10,
        std::bind(&PlannerNode::on_vehicle_speed, this, std::placeholders::_1));
    // - TARGET LOCATION
    target_location_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_location", 10,
        std::bind(&PlannerNode::on_target_location, this,
                  std::placeholders::_1));

    // # Publishers
    // - SPEED PROFILE
    speed_profile_pub_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "speed_profile", 10);
    // - TARGET PATH
    target_path_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "target_path", 10);

    // # Create Planning Loop
    // fire at rate_hz
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&PlannerNode::planning_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized");
  }

private:
  // # Callbacks
  void on_hd_map(const std_msgs::msg::Float32MultiArray::SharedPtr) {
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received HD Map");
  }

  void on_turn_angle(const std_msgs::msg::Float32::SharedPtr) {
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received current car turn angle");
  }

  void on_vehicle_speed(const std_msgs::msg::Float32::SharedPtr) {
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received current vehicle speed");
  }

  void on_target_location(const geometry_msgs::msg::Point::SharedPtr msg) {
    // todo: implement
    RCLCPP_INFO(this->get_logger(),
                "Target location received: (%.2f, %.2f, %.2f)", msg->x, msg->y,
                msg->z);
  }

  /**
   * Every loop here the
   */
  void planning_loop_callback() {
    // no log message for each loop
    // send route msg to ctrl
    // send speed msg to ctrl
  }

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
