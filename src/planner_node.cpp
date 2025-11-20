#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

#include "ap1/planning/planner_node.hpp"

using ap1_msgs::msg::SpeedProfileStamped;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::TurnAngleStamped;
using ap1_msgs::msg::VehicleSpeedStamped;
using geometry_msgs::msg::Point;
using std_msgs::msg::Float32MultiArray;

namespace ap1::planning
{
PlannerNode::PlannerNode(double rate_hz) : Node("planner_node"), rate_hz_(rate_hz)
{
    // # Subscribe to all inputs
    // todo: paths should be loaded from config
    // - HD MAP WRONG TYPE FOR NOW
    hd_map_sub_ = this->create_subscription<Float32MultiArray>(
        "/ap1/map/full_had_map", 10,
        std::bind(&PlannerNode::on_hd_map, this, std::placeholders::_1));
    vehicle_speed_sub_ = this->create_subscription<VehicleSpeedStamped>(
        "/ap1/actuation/speed_actual", 10,
        std::bind(&PlannerNode::on_vehicle_speed, this, std::placeholders::_1));
    target_location_sub_ = this->create_subscription<Point>(
        "/ap1/control/target_location", 10,
        std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));
    target_speed_sub_ = this->create_subscription<VehicleSpeedStamped>(
        "/ap1/control/target_speed", 10,
        std::bind(&PlannerNode::on_target_speed, this, std::placeholders::_1));

    // # Publishers
    speed_profile_pub_ =
        this->create_publisher<SpeedProfileStamped>("/ap1/planning/speed_profile", 10);
    target_path_pub_ = this->create_publisher<TargetPathStamped>("/ap1/planning/target_path", 10);

    // # Create Planning Loop @ rate_hz
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0f / rate_hz),
                                     std::bind(&PlannerNode::planning_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized.");
}

// # Callbacks
void PlannerNode::on_hd_map(const Float32MultiArray::SharedPtr)
{
    // todo: implement
}

void PlannerNode::on_turn_angle(const TurnAngleStamped::SharedPtr)
{
    // todo: implement
}

void PlannerNode::on_vehicle_speed(const VehicleSpeedStamped::SharedPtr)
{
    // todo: implement
}

void PlannerNode::on_target_speed(const VehicleSpeedStamped::SharedPtr msg)
{
    this->speed_ = msg->speed;

    std::string s = "Command: set speed to " + std::to_string(this->speed_);
    RCLCPP_INFO(this->get_logger(), s.c_str());
}

void PlannerNode::on_target_location(const Point::SharedPtr msg)
{
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Target location received: (%.2f, %.2f, %.2f)", msg->x, msg->y,
                msg->z);
}

TargetPathStamped PlannerNode::create_route()
{
    // create msg
    TargetPathStamped path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id =
        "map"; // this has something to do with coordinate systems but idk tbh

    // fill path. create 10 points each 1m apart, directly forward
    path_msg.path = {};
    for (int i = 1; i <= 10; i++)
    {
        // +x is forward, +y is left
        Point p;
        p.x = i * 1.0;
        p.y = 0.0;
        // points[i] = p;
        path_msg.path.push_back(p);
    }

    return path_msg;
}

SpeedProfileStamped PlannerNode::create_speed_profile()
{
    SpeedProfileStamped speed_msg;

    speed_msg.header.stamp = this->now();
    speed_msg.header.frame_id = "map";

    // set the speed to the system's last recieved target speed
    speed_msg.speeds = {this->speed_};

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
