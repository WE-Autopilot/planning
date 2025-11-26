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

    // Declare map file path parameter (can be set via launch file or command line)
    this->declare_parameter<std::string>("map_file_path", "");
    map_file_path_ = this->get_parameter("map_file_path").as_string();

    process_map_data();

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

void PlannerNode::process_map_data()
{
    current_lane_.left_boundary.clear();
    current_lane_.right_boundary.clear();

    // If no map file provided, use mock data for testing
    if (map_file_path_.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "No map file provided. Using mock curved road data. "
                    "Set 'map_file_path' parameter to load a real Lanelet2 map.");

        // Mock a curved road (quadratic: y = 0.05 * x^2)
        for (int i = 0; i <= 20; i++)
        {
            double x = i * 1.0;
            double y_center = 0.05 * x * x;

            geometry_msgs::msg::Point p_left;
            p_left.x = x;
            p_left.y = y_center + 1.5;
            p_left.z = 0.0;
            current_lane_.left_boundary.push_back(p_left);

            geometry_msgs::msg::Point p_right;
            p_right.x = x;
            p_right.y = y_center - 1.5;
            p_right.z = 0.0;
            current_lane_.right_boundary.push_back(p_right);
        }
        return;
    }

    // Load Lanelet2 map from file
    try
    {
        // Use UTM projection (adjust origin to your map's location)
        // TODO: Make origin configurable via parameters
        lanelet::projection::UtmProjector projector(lanelet::Origin({49.0, 8.4}));
        lanelet_map_ = lanelet::load(map_file_path_, projector);

        RCLCPP_INFO(this->get_logger(), "Loaded Lanelet2 map from: %s", map_file_path_.c_str());

        // Extract first lanelet from the map
        if (lanelet_map_->laneletLayer.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Loaded map contains no lanelets!");
            return;
        }

        lanelet::Lanelet lanelet = *lanelet_map_->laneletLayer.begin();
        RCLCPP_INFO(this->get_logger(), "Using lanelet ID: %ld", lanelet.id());

        // Extract left and right boundaries
        lanelet::ConstLineString3d left_bound = lanelet.leftBound();
        lanelet::ConstLineString3d right_bound = lanelet.rightBound();

        // Convert Lanelet2 points to geometry_msgs::Point
        for (const auto& pt : left_bound)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            current_lane_.left_boundary.push_back(p);
        }

        for (const auto& pt : right_bound)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            current_lane_.right_boundary.push_back(p);
        }

        RCLCPP_INFO(this->get_logger(),
                    "Extracted lane with %zu left and %zu right boundary points",
                    current_lane_.left_boundary.size(), current_lane_.right_boundary.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s. Using mock data.", e.what());
        // Fall back to mock data
        for (int i = 0; i <= 20; i++)
        {
            double x = i * 1.0;
            double y_center = 0.05 * x * x;

            geometry_msgs::msg::Point p_left;
            p_left.x = x;
            p_left.y = y_center + 1.5;
            p_left.z = 0.0;
            current_lane_.left_boundary.push_back(p_left);

            geometry_msgs::msg::Point p_right;
            p_right.x = x;
            p_right.y = y_center - 1.5;
            p_right.z = 0.0;
            current_lane_.right_boundary.push_back(p_right);
        }
    }
}

std::vector<geometry_msgs::msg::Point> PlannerNode::calculate_centerline(const Lane& lane)
{
    std::vector<geometry_msgs::msg::Point> centerline;

    if (lane.left_boundary.size() != lane.right_boundary.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Left and right lane boundaries have different sizes!");
        return centerline;
    }

    for (size_t i = 0; i < lane.left_boundary.size(); ++i)
    {
        geometry_msgs::msg::Point p;
        p.x = (lane.left_boundary[i].x + lane.right_boundary[i].x) / 2.0;
        p.y = (lane.left_boundary[i].y + lane.right_boundary[i].y) / 2.0;
        p.z = 0.0; // Assuming flat ground for now
        centerline.push_back(p);
    }

    return centerline;
}

TargetPathStamped PlannerNode::create_route()
{
    // create msg
    TargetPathStamped path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id =
        "map"; // this has something to do with coordinate systems but idk tbh

    // fill path. create 10 points each 1m apart, directly forward
    path_msg.path = calculate_centerline(current_lane_);

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
