cmake_minimum_required(VERSION 3.20)
project(neo_teleop2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(neo_teleop2_node src/neo_teleop.cpp)

ament_target_dependencies(neo_teleop2_node
  rclcpp
  sensor_msgs
  geometry_msgs)

add_executable(ur_teleop_node
  src/ur_teleop/main.cpp
  src/ur_teleop/ur_teleop_commands.cpp
  src/ur_teleop/ur_teleop_joy.cpp
  src/ur_teleop/ur_teleop_moveit.cpp
  src/ur_teleop/ur_teleop_node.cpp
)
target_compile_features(ur_teleop_node PRIVATE cxx_std_23)
ament_target_dependencies(ur_teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  controller_manager_msgs
  moveit_servo
  moveit_ros_planning_interface
)


install(TARGETS neo_teleop2_node ur_teleop_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
