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
        this->target_location_ = *msg; //Update target
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

        // TODO: Get waypoints from mapping/localization team when available
        //
        // ASSUMPTIONS:
        // - navigable_waypoints represents all valid waypoints in the navigable space (e.g., road network)
        // - The order of waypoints in the vector does NOT represent the path order
        // - The navigation logic (locate_closest_waypoint + generate_waypoint_sequence) determines
        //   which waypoints to navigate to based on the target location
        // - Waypoints should be treated as an unordered set of valid positions, not a sequential path
        // - Final mapping implementation will provide these waypoints; structure/format TBD
        //
        // Test path: Spiral pattern - up 9, right 9, down 6, left 6, up 3, right 3
        std::vector<vec2f> navigable_waypoints = {
            // Up 9: (0,0) to (0,9)
            vec2f(0.0f, 0.0f), vec2f(0.0f, 1.0f), vec2f(0.0f, 2.0f), vec2f(0.0f, 3.0f), vec2f(0.0f, 4.0f),
            vec2f(0.0f, 5.0f), vec2f(0.0f, 6.0f), vec2f(0.0f, 7.0f), vec2f(0.0f, 8.0f), vec2f(0.0f, 9.0f),
            // Right 9: (0,9) to (9,9)
            vec2f(1.0f, 9.0f), vec2f(2.0f, 9.0f), vec2f(3.0f, 9.0f), vec2f(4.0f, 9.0f), vec2f(5.0f, 9.0f),
            vec2f(6.0f, 9.0f), vec2f(7.0f, 9.0f), vec2f(8.0f, 9.0f), vec2f(9.0f, 9.0f),
            // Down 6: (9,9) to (9,3)
            vec2f(9.0f, 8.0f), vec2f(9.0f, 7.0f), vec2f(9.0f, 6.0f), vec2f(9.0f, 5.0f), vec2f(9.0f, 4.0f), vec2f(9.0f, 3.0f),
            // Left 6: (9,3) to (3,3)
            vec2f(8.0f, 3.0f), vec2f(7.0f, 3.0f), vec2f(6.0f, 3.0f), vec2f(5.0f, 3.0f), vec2f(4.0f, 3.0f), vec2f(3.0f, 3.0f),
            // Up 3: (3,3) to (3,6)
            vec2f(3.0f, 4.0f), vec2f(3.0f, 5.0f), vec2f(3.0f, 6.0f),
            // Right 3: (3,6) to (6,6)
            vec2f(4.0f, 6.0f), vec2f(5.0f, 6.0f), vec2f(6.0f, 6.0f)
        };

        // Convert target location from ROS Point message to vec2f
        vec2f target_vec2f(target_location_.x, target_location_.y);

        // Step 1: Find which waypoint is closest to the target destination
        // This is like dropping a pin on Google Maps - it finds the nearest road
        size_t closest_waypoint_idx = locate_closest_waypoint(target_vec2f, navigable_waypoints);

        // Step 2: Create fallback path for when no waypoints are available
        // Fallback = just go directly to the target (like driving in a parking lot)
        std::vector<vec2f> fallback_path = { target_vec2f };

        // Step 3: Generate the actual waypoint sequence the car should follow
        // If waypoints exist: follows the lane/road waypoints
        // If no waypoints: uses fallback path (direct to target)
        std::vector<vec2f> waypoint_sequence = generate_waypoint_sequence(
            navigable_waypoints,
            closest_waypoint_idx,
            fallback_path
        );

        // Step 4: Convert vec2f waypoints back to ROS Point messages
        path_msg.path = {};
        for (const auto& waypoint : waypoint_sequence) {
            Point p;
            p.x = waypoint.x;
            p.y = waypoint.y;
            p.z = 0.0;  // Ground vehicle, so z is always 0
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
