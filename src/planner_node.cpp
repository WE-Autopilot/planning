#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "ap1/planning/planner_node.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"

namespace ap1::planning
{
PlannerNode::PlannerNode(double rate_hz) : Node("planner_node"), rate_hz_(rate_hz)
{
    // # Subscribe to all inputs
    // - HD MAP
    hd_map_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "ap1/map/full_had_map", 10,
        std::bind(&PlannerNode::on_hd_map, this, std::placeholders::_1));
    // - TURN ANGLE
    turn_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "ap1/actuation/turn_angle_actual", 10,
        std::bind(&PlannerNode::on_turn_angle, this, std::placeholders::_1));
    // - VEHICLE SPEED
    vehicle_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "ap1/actuation/speed_actual", 10,
        std::bind(&PlannerNode::on_vehicle_speed, this, std::placeholders::_1));
    // - TARGET LOCATION
    target_location_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "ap1/control/target_location", 10,
        std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));

    // # Publishers
    // - SPEED PROFILE
    speed_profile_pub_ = this->create_publisher<ap1_msgs::msg::SpeedProfileStamped>(
        "ap1/planning/speed_profile", 10);
    // - TARGET PATH
    target_path_pub_ =
        this->create_publisher<ap1_msgs::msg::TargetPathStamped>("ap1/planning/target_path", 10);

    // # Create Planning Loop
    // fire at rate_hz
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                     std::bind(&PlannerNode::planning_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized");
}

// # Callbacks
void PlannerNode::on_hd_map(const std_msgs::msg::Float32MultiArray::SharedPtr)
{
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received HD Map");
}

void PlannerNode::on_turn_angle(const std_msgs::msg::Float32::SharedPtr)
{
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received current car turn angle");
}

void PlannerNode::on_vehicle_speed(const std_msgs::msg::Float32::SharedPtr)
{
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received current vehicle speed");
}

void PlannerNode::on_target_location(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Target location received: (%.2f, %.2f, %.2f)", msg->x, msg->y,
                msg->z);
}

ap1_msgs::msg::TargetPathStamped PlannerNode::create_route()
{
    ap1_msgs::msg::TargetPathStamped path_msg;

    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    // fill path
    path_msg.path = {};

    // create 10 points each 1m apart, directly forward
    for (int i = 1; i <= 10; i++)
    {
        geometry_msgs::msg::Point p;
        p.x = 0.0;
        p.y = i * 1.0;
        // points[i] = p;
        path_msg.path.push_back(p);
    }

    return path_msg;
}

ap1_msgs::msg::SpeedProfileStamped PlannerNode::create_speed_profile()
{
    ap1_msgs::msg::SpeedProfileStamped speed_msg;

    speed_msg.header.stamp = this->now();
    speed_msg.header.frame_id = "map";

    // set the speed to 1 m/s
    speed_msg.speeds = {1.0};

    return speed_msg;
}

void PlannerNode::planning_loop_callback()
{
    // no log message for each loop

    // send route msg to ctrl
    target_path_pub_->publish(create_route());

    // send speed msg to ctrl
    speed_profile_pub_->publish(create_speed_profile());
}
} // namespace ap1::planning
