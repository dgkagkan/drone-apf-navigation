cmake_minimum_required(VERSION 3.8)
project(commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(vtol_tf_broadcaster src/vtol_tf_broadcaster.cpp)
ament_target_dependencies(vtol_tf_broadcaster
  rclcpp px4_msgs geometry_msgs tf2 tf2_ros)

add_executable(vtol_offboard_control src/vtol_offboard_control.cpp)
ament_target_dependencies(vtol_offboard_control
  rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(mission_manager src/mission_manager.cpp)
ament_target_dependencies(mission_manager
  rclcpp px4_msgs geometry_msgs sensor_msgs std_msgs)

install(TARGETS
  vtol_tf_broadcaster
  vtol_offboard_control
  mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
