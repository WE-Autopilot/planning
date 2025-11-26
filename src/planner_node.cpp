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

double computeCurvature(const geometry_msgs::msg::Point &p0, const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2){
    double x1 = p0.x, y1 = p0.y;
    double x2 = p1.x, y2 = p1.y;
    double x3 = p2.x, y3 = p2.y;

    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;

    double cross = dx1 * dy2 - dy1 * dx2;
    double dot   = dx1 * dx2 + dy1 * dy2;

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
    speeds.front() = std::min(MAX_SPEED_MPS, this->speed_);
    speeds.back()  = speeds[speeds.size() - 2];

    speed_msg.speeds = speeds;

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
