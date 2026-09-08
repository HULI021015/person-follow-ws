cmake_minimum_required(VERSION 3.16)
project(piper_kinematics_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(piper_ik_node src/piper_ik_node.cpp)
target_include_directories(piper_ik_node PRIVATE include/piper_kinematics_ros2)
target_link_libraries(piper_ik_node Eigen3::Eigen)
ament_target_dependencies(
  piper_ik_node rclcpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros)

install(TARGETS piper_ik_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
