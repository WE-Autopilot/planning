/**
 * @brief Planner node
 * @version 0.1.0
 * @date 2025-11-26

 * CAVEATS:
 * - Loads map ONLY ONCE at the beginning of the runtime.
 * - assumes left-right lane lines have same number of waypoints
 * - assumes left-right lane lines are directly accross from eachother.
 *
 * We should assemble a smooth curve out of waypoints and determine centerline based on that
 * instead.
 */

#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"

#include "ap1/planning/planner_node.hpp"

using geometry_msgs::msg::Point;

using ap1_msgs::msg::SpeedProfileStamped;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::LaneBoundaries;

namespace ap1::planning
{
PlannerNode::PlannerNode(double rate_hz) : Node("planner_node"), rate_hz_(rate_hz)
{
    // # Subscribe to all inputs
    // todo: paths should be loaded from config
    this->lane_sub_ = create_subscription<LaneBoundaries>(
        "/ap1/mapping/lanes", 1,
        std::bind(&PlannerNode::on_lanes, this, std::placeholders::_1));
    this->target_location_sub_ = create_subscription<Point>(
        "/ap1/control/target_location", 1,
        std::bind(&PlannerNode::on_target_location, this, std::placeholders::_1));
    this->target_speed_sub_ = create_subscription<FloatStamped>(
        "/ap1/control/target_speed", 1,
        std::bind(&PlannerNode::on_target_speed, this, std::placeholders::_1));

    // # Publishers
    this->speed_profile_pub_ = create_publisher<SpeedProfileStamped>("/ap1/planning/speed_profile", 1);
    this->target_path_pub_ = create_publisher<TargetPathStamped>("/ap1/planning/target_path", 1);

    // # Create Planning Loop @ rate_hz
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0f / rate_hz),
        std::bind(&PlannerNode::planning_loop_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized.");
}

// # Callbacks
void PlannerNode::on_lanes(const LaneBoundaries::SharedPtr lane)
{
    this->current_lane_ = lane;
}

void PlannerNode::on_target_speed(const FloatStamped::SharedPtr msg)
{
    this->speed_ = msg->value;

    std::string s = "Command: set speed to " + std::to_string(this->speed_);
    RCLCPP_INFO_STREAM(this->get_logger(), s);
}

void PlannerNode::on_target_location(const Point::SharedPtr msg)
{
    this->target_location_.x = msg->x;
    this->target_location_.y = msg->y;

    RCLCPP_INFO(this->get_logger(), "Target location received: (%.2f, %.2f, %.2f)", msg->x, msg->y,
                msg->z);
}

std::vector<vec2f> PlannerNode::calculate_centerline(const LaneBoundaries::SharedPtr lane)
{
    std::vector<vec2f> centerline;

    if (lane->left.size() != lane->right.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Left and right lane boundaries have different sizes!");
        return centerline;
    }

    for (size_t i = 0; i < lane->left.size(); ++i)
    {
        centerline.emplace_back((lane->left[i].x + lane->right[i].x) / 2.0,
                                (lane->left[i].y + lane->right[i].y) / 2.0);
    }

    return centerline;
}

TargetPathStamped PlannerNode::create_route()
{
    // create msg
    TargetPathStamped path_msg;
    path_msg.header.stamp = this->now();

    // calculate centerline
    if (!this->current_lane_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current lane is null.");
        path_msg.path = {};
        return path_msg;
    }
    auto centerline = calculate_centerline(this->current_lane_);
    if (centerline.empty()) RCLCPP_WARN(this->get_logger(), "Centerline is empty!");

    // Convert target location from ROS Point message to vec2f
    vec2f target_vec2f(target_location_.x, target_location_.y);

    // Step 1: Find which waypoint in the centerline is closest to the target destination
    // This is like dropping a pin on Google Maps - it finds the nearest road
    int closest_waypoint_idx = locate_closest_waypoint(target_vec2f, centerline);
    if (closest_waypoint_idx == -1) RCLCPP_WARN(this->get_logger(), "Closest waypoint calculation: IDX=-1!");

    // Step 2: Create fallback path for when no waypoints are available
    // Fallback = just go directly to the target (like driving in a parking lot)
    std::vector<vec2f> fallback_path = {target_vec2f};

    // Step 3: Generate the actual waypoint sequence the car should follow
    // If waypoints exist: follows the lane/road waypoints
    // If no waypoints: uses fallback path (direct to target)
    std::vector<vec2f> waypoint_sequence = generate_waypoint_sequence(centerline, closest_waypoint_idx, fallback_path);

    // Step 4: Convert vec2f waypoints back to ROS Point messages
    path_msg.path = {};
    for (const auto& waypoint : waypoint_sequence)
    {
        Point p;
        p.x = waypoint.x;
        p.y = waypoint.y;
        p.z = 0.0; // Ground vehicle, so z is always 0
        path_msg.path.push_back(p);
    }

    return path_msg;
}

SpeedProfileStamped PlannerNode::create_speed_profile()
{
    SpeedProfileStamped speed_msg;

    // set the speed to the system's last recieved target speed
    speed_msg.speeds = {this->speed_};

    return speed_msg;
}

void PlannerNode::planning_loop_callback()
{
    // send route msg to ctrl
    target_path_pub_->publish(create_route());

    // send speed msg to ctrl
    speed_profile_pub_->publish(create_speed_profile());
}
} // namespace ap1::planning
