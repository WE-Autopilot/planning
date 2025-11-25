#include "ap1/planning/planner_node.hpp"
#include "ap1/planning/math_utils.hpp"
#include <iostream>

// should ideally provide rate_hz parsed from config file
// leaving this for when launch files are created
const float RATE_HZ = 60;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ap1::planning::PlannerNode>(RATE_HZ);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
