cmake_minimum_required(VERSION 3.8)
project(autonomy_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(autonomy_node src/autonomy_node.cpp)

ament_target_dependencies(autonomy_node rclcpp sensor_msgs)

install(TARGETS autonomy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()