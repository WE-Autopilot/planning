/**
 * @file planner_node.cpp
 * @brief Planner node
 * @version 0.1
 * @date 2025-11-26
 *
 * @copyright Copyright (c) 2025
 *
 * CAVEATS:
 * - Loads map ONLY ONCE at the beginning of the runtime.
 * - assumes left-right lane lines have same number of waypoints
 * - assumes left-right lane lines are directly accross from eachother.
 *
 * We should assemble a smooth curve out of waypoints and determine centerline based on that
 * instead.
 */

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"


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

// ===================== SPEED PROFILE CONSTANTS ===================== //

constexpr double MAX_SPEED_MPS = 6.0;         // Global max speed
constexpr double MAX_LAT_ACC_MPS2 = 1.5;      // Lateral accel limit (turning)
constexpr double MAX_FWD_ACC_MPS2 = 2.0;      // Forward accel limit
constexpr double MIN_TURN_RADIUS = 0.01;      // Avoid division by zero

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
    speed_limit_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ap1/planning/_current_speed_limit", 10);

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
    RCLCPP_INFO_STREAM(this->get_logger(), s);
}

void PlannerNode::on_target_location(const Point::SharedPtr msg)
{
    this->target_location_.x = msg->x;
    this->target_location_.y = msg->y;

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
        for (int i = 1; i <= 20; i++)
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
        RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s. Defaulting to direct navigation.",
                     e.what());
    }
}

std::vector<vec2f> PlannerNode::calculate_centerline(const Lane& lane)
{
    std::vector<vec2f> centerline;

    if (lane.left_boundary.size() != lane.right_boundary.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Left and right lane boundaries have different sizes!");
        return centerline;
    }

    for (size_t i = 0; i < lane.left_boundary.size(); ++i)
    {
        centerline.emplace_back((lane.left_boundary[i].x + lane.right_boundary[i].x) / 2.0,
                                (lane.left_boundary[i].y + lane.right_boundary[i].y) / 2.0);
    }

    return centerline;
}

TargetPathStamped PlannerNode::create_route()
{
    // create msg
    TargetPathStamped path_msg;
    path_msg.header.stamp = this->now();

    // calculate centerline
    auto centerline = calculate_centerline(this->current_lane_);
    if (centerline.empty()) RCLCPP_WARN(this->get_logger(), "Centerline is empty!");

    // Convert target location from ROS Point message to vec2f
    vec2f target_vec2f(target_location_.x, target_location_.y);

    // Step 1: Find which waypoint in the centerline is closest to the target destination
    // This is like dropping a pin on Google Maps - it finds the nearest road
    int closest_waypoint_idx = locate_closest_waypoint(
        target_vec2f, centerline);
    if (closest_waypoint_idx == -1) RCLCPP_WARN(this->get_logger(), "Closest waypoint calculation: IDX=-1!");

    // Step 2: Create fallback path for when no waypoints are available
    // Fallback = just go directly to the target (like driving in a parking lot)
    std::vector<vec2f> fallback_path = {target_vec2f};

    // Step 3: Generate the actual waypoint sequence the car should follow
    // If waypoints exist: follows the lane/road waypoints
    // If no waypoints: uses fallback path (direct to target)
    std::vector<vec2f> waypoint_sequence =
        generate_waypoint_sequence(centerline, closest_waypoint_idx, fallback_path);

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

double computeCurvature(const geometry_msgs::msg::Point &p0, const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2){
    double x1 = p0.x, y1 = p0.y;
    double x2 = p1.x, y2 = p1.y;
    double x3 = p2.x, y3 = p2.y;

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;

    double cross = dx1 * dy2 - dy1 * dx2;
    [[maybe_unused]] double dot = dx1 * dx2 + dy1 * dy2;
    double denom = std::sqrt((dx1*dx1 + dy1*dy1) * (dx2*dx2 + dy2*dy2));

    if (denom < 1e-6)
        return 0.0;

    return std::abs(cross) / denom;
}


SpeedProfileStamped PlannerNode::create_speed_profile()
{
    SpeedProfileStamped speed_msg;

    speed_msg.header.stamp = this->now();
    speed_msg.header.frame_id = "map";

    // =======================================================================
    // 1. Get the path (same one sent in target_path_pub_)
    // =======================================================================
    TargetPathStamped route = create_route();
    const auto &pts = route.path;

    std::vector<double> speeds;
    speeds.resize(pts.size(), MAX_SPEED_MPS);

    // =======================================================================
    // 2. Dynamic turn-based speed profiling
    // =======================================================================
    double last_speed = this->speed_;   // previously commanded speed
    double dt = 0.1;                    // assume 10 Hz planning loop

    for (size_t i = 1; i + 1 < pts.size(); i++)
    {
        const auto &p0 = pts[i - 1];
        const auto &p1 = pts[i];
        const auto &p2 = pts[i + 1];

        // ---- curvature → radius ------------------------------------------
        double curvature = computeCurvature(p0, p1, p2);
        double R = (curvature == 0.0) ? 1e9 : 1.0 / curvature;

        // ---- lateral-acceleration speed limit -----------------------------
        double turn_speed_limit = std::sqrt(MAX_LAT_ACC_MPS2 * std::max(R, MIN_TURN_RADIUS));

        // ---- combine with global limit ------------------------------------
        double target_speed = std::min(MAX_SPEED_MPS, turn_speed_limit);

        // ---- forward acceleration limit -----------------------------------
        double dv = target_speed - last_speed;
        double max_dv = MAX_FWD_ACC_MPS2 * dt;

        if (dv > max_dv) target_speed = last_speed + max_dv;
        if (dv < -max_dv) target_speed = last_speed - max_dv;

        // ---- final safety clamp -------------------------------------------
        if (target_speed > MAX_SPEED_MPS)
            target_speed = MAX_SPEED_MPS;

        double lat_acc = (target_speed * target_speed) / std::max(R, MIN_TURN_RADIUS);
        if (lat_acc > MAX_LAT_ACC_MPS2)
            target_speed = std::sqrt(MAX_LAT_ACC_MPS2 * R);

        speeds[i] = target_speed;
        last_speed = target_speed;
    }

    // Set first/last speeds
    speeds.front() = std::min(MAX_SPEED_MPS, static_cast<double>(this->speed_));
    speeds.back()  = speeds[speeds.size() - 2];
    
    // -------------------------------
//  FINAL SAFETY CHECK (required)
// -------------------------------
for (size_t i = 1; i + 1 < speeds.size(); i++)
{
    double v = speeds[i];

    if (v > MAX_SPEED_MPS)
        v = MAX_SPEED_MPS;

    const auto &p0 = pts[i - 1];
    const auto &p1 = pts[i];
    const auto &p2 = pts[i + 1];

    double curvature = computeCurvature(p0, p1, p2);
    double R = (curvature == 0.0) ? 1e9 : 1.0 / curvature;
    double max_v_turn = std::sqrt(MAX_LAT_ACC_MPS2 * R);

    if (v > max_v_turn)
        v = max_v_turn;

    speeds[i] = v;
}

    speed_msg.speeds.clear();
    speed_msg.speeds.reserve(speeds.size());

    for (double v : speeds) {
        speed_msg.speeds.push_back(static_cast<float>(v));
    }

    return speed_msg;
}


void PlannerNode::planning_loop_callback()
{
    // no log message for each loop

    // 1. Create route
    auto route = create_route();
    target_path_pub_->publish(route);

    // 2. Create dynamic speed profile
    auto speed_profile = create_speed_profile();
    speed_profile_pub_->publish(speed_profile);

    // 3. Publish current speed limit (for UI)
    // Use the first waypoint's computed limit.
    std_msgs::msg::Float64 limit_msg;
    if (!speed_profile.speeds.empty()) {
        limit_msg.data = speed_profile.speeds.front();
    } else {
        limit_msg.data = MAX_SPEED_MPS;   // fallback
    }
    speed_limit_pub_->publish(limit_msg);
}

} // namespace ap1::planning
