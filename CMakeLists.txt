cmake_minimum_required(VERSION 3.8)
project(ap1_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ap1_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

add_executable(planner_node
  src/main.cpp
  src/planner_node.cpp
  src/math_utils.cpp
  src/waypoint_utils.cpp
)

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planner_node
  rclcpp
  std_msgs
  geometry_msgs
  ap1_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # disable linting for now bc uncrustify style checks conflict with my clang configs
  # use the clang-format file to format your code 
endif()

ament_package()
