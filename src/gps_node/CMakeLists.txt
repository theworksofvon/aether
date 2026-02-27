cmake_minimum_required(VERSION 3.8)
project(gps_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gps_node src/gps_node.cpp)

ament_target_dependencies(gps_node rclcpp sensor_msgs)

install(TARGETS gps_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()