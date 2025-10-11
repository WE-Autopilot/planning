/**
 * Created: Oct. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_PLANNING_NODE_HPP
#define AP1_PLANNING_NODE_HPP

#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace ap1::planning {
    class PlannerNode : public rclcpp::Node {
    public:
        PlannerNode() : Node("planner_node") {
            // # Subscribe to all inputs
            // - HD MAP
            hd_map_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "hd_map", 10, std::bind(&PlannerNode::on_hd_map, this, std::placeholders::_1));
            // - TURN ANGLE
            turn_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                    "turn_angle", 10, std::bind(&PlannerNode::on_turn_angle, this, std::placeholders::_1));
            // - VEHICLE SPEED
            vehicle_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                    "vehicle_speed", 10, std::bind(&PlannerNode::on_vehicle_speed, this, std::placeholders::_1));
            // - TARGET LOCATION
            target_location_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                    "target_location", 10, std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));

            // # Publishers
            // - SPEED PROFILE
            speed_profile_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("speed_profile", 10);
            // - TARGET PATH 
            target_path_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_path", 10);

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
            RCLCPP_INFO(this->get_logger(), "Target location received: (%.2f, %.2f, %.2f)",
                    msg->x, msg->y, msg->z);
        }

        // # Subscriptions
        // Note: use SharedPtrs for all messages since they're dynamically allocated in ROS
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hd_map_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr turn_angle_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vehicle_speed_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_location_sub_;

        // # Publishers
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_path_pub_;
    };
}

#endif // AP1_PLANNING_NODE_HPP
