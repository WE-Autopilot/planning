#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "ap1/planning/planner_node.hpp"

namespace ap1::planning {
PlannerNode::PlannerNode(double rate_hz)
    : Node("planner_node"), rate_hz_(rate_hz) {
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
      std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));

  // # Publishers
  // - SPEED PROFILE
  speed_profile_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
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

// # Callbacks
void PlannerNode::on_hd_map(const std_msgs::msg::Float32MultiArray::SharedPtr) {
  // todo: implement
  RCLCPP_INFO(this->get_logger(), "Received HD Map");
}

void PlannerNode::on_turn_angle(const std_msgs::msg::Float32::SharedPtr) {
  // todo: implement
  RCLCPP_INFO(this->get_logger(), "Received current car turn angle");
}

void PlannerNode::on_vehicle_speed(const std_msgs::msg::Float32::SharedPtr) {
  // todo: implement
  RCLCPP_INFO(this->get_logger(), "Received current vehicle speed");
}

void PlannerNode::on_target_location(
    const geometry_msgs::msg::Point::SharedPtr msg) {
  // todo: implement
  RCLCPP_INFO(this->get_logger(),
              "Target location received: (%.2f, %.2f, %.2f)", msg->x, msg->y,
              msg->z);
}

void PlannerNode::planning_loop_callback() {
  // no log message for each loop
  // send route msg to ctrl
  // send speed msg to ctrl
}
} // namespace ap1::planning
