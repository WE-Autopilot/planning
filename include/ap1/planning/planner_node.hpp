/**
 * Created: Oct. 11, 2025
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
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <std_srvs/srv/trigger.hpp>

namespace ap1::planning
{
    class PlannerNode : public rclcpp::Node
    {
    public:
        PlannerNode()
            : Node("planner_node")
        {
            // CONFIG PATH
            config_path_ =
                std::string(
                    std::filesystem::path(__FILE__)
                        .parent_path()
                        .parent_path()
                        .parent_path()
                        .parent_path()
                ) + "/config.yaml";

            // Load config
            load_config();

            // SUBSCRIPTIONS
            hd_map_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                "hd_map", 10, std::bind(&PlannerNode::on_hd_map, this, std::placeholders::_1));

            turn_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
                "turn_angle", 10, std::bind(&PlannerNode::on_turn_angle, this, std::placeholders::_1));

            vehicle_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
                "vehicle_speed", 10, std::bind(&PlannerNode::on_vehicle_speed, this, std::placeholders::_1));

            target_location_sub_ = create_subscription<geometry_msgs::msg::Point>(
                "target_location", 10, std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));

            // PUBLISHERS
            speed_profile_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("speed_profile", 10);
            target_path_pub_   = create_publisher<std_msgs::msg::Float32MultiArray>("target_path", 10);

            // RESET SERVICE
            reset_service_ = create_service<std_srvs::srv::Trigger>(
                "/planning/reset",
                std::bind(&PlannerNode::handle_reset, this,
                          std::placeholders::_1, std::placeholders::_2));

            // TIMER
            timer_ = create_wall_timer(
                std::chrono::milliseconds(1000 / rate_hz_),
                std::bind(&PlannerNode::on_timer, this));

            RCLCPP_INFO(get_logger(), "PlannerNode running at %d Hz", rate_hz_);
        }

    private:
        int rate_hz_;
        std::string config_path_;
        rclcpp::TimerBase::SharedPtr timer_;

        // ---- CONFIG LOADING ----
        void load_config()
        {
            YAML::Node config = YAML::LoadFile(config_path_);

            if (!config["rate_hz"])
                throw std::runtime_error("Missing rate_hz in planning config");

            rate_hz_ = config["rate_hz"].as<int>();

            RCLCPP_INFO(this->get_logger(),
                        "[Planning] Config reloaded: rate_hz=%d", rate_hz_);
        }

        // ---- TIMER ----
        void on_timer()
        {
            // Fake speed increasing
            if (speed_profile_.empty())
                speed_profile_.push_back(0.0f);
            else
            {
                speed_profile_[0] += 0.5f;
                if (speed_profile_[0] > 30.0f)
                    speed_profile_[0] = 0.0f;
            }

            std_msgs::msg::Float32MultiArray speed_msg;
            speed_msg.data = speed_profile_;
            speed_profile_pub_->publish(speed_msg);

            // Fake path: just push target
            std_msgs::msg::Float32MultiArray path_msg;
            path_msg.data.push_back(target_x_);
            path_msg.data.push_back(target_y_);
            target_path_pub_->publish(path_msg);
        }

        // ---- SUBS ----
        void on_hd_map(const std_msgs::msg::Float32MultiArray::SharedPtr) {}
        void on_turn_angle(const std_msgs::msg::Float32::SharedPtr) {}
        void on_vehicle_speed(const std_msgs::msg::Float32::SharedPtr) {}

        void on_target_location(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            target_x_ = msg->x;
            target_y_ = msg->y;
        }

        // ---- RESET ----
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

        void handle_reset(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            speed_profile_.clear();
            planned_path_.clear();
            target_x_ = 0.0;
            target_y_ = 0.0;

            load_config();  // RELOAD CONFIG HERE ✔

            response->success = true;
            response->message = "Planning reset + config reloaded";

            RCLCPP_INFO(get_logger(), "Planning reset completed.");
        }

        // ---- STATE ----
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hd_map_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr turn_angle_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vehicle_speed_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_location_sub_;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_path_pub_;

        std::vector<float> speed_profile_;
        std::vector<std::pair<float, float>> planned_path_;
        float target_x_ = 0.0;
        float target_y_ = 0.0;
    };
}

#endif
