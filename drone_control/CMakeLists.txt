cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(automated_mission_node src/automated_mission_node.cpp)
target_compile_features(automated_mission_node PRIVATE cxx_std_17)
ament_target_dependencies(automated_mission_node
  drone_interfaces rclcpp sensor_msgs)

add_executable(flight_supervisor_node src/flight_supervisor_node.cpp)
target_compile_features(flight_supervisor_node PRIVATE cxx_std_17)
ament_target_dependencies(flight_supervisor_node
  drone_interfaces px4_msgs rclcpp rclcpp_action)

add_executable(gimbal_control_node src/gimbal_control_node.cpp)
target_compile_features(gimbal_control_node PRIVATE cxx_std_17)
ament_target_dependencies(gimbal_control_node
  rclcpp sensor_msgs std_msgs)

add_executable(manual_control_node src/manual_control_node.cpp)
target_compile_features(manual_control_node PRIVATE cxx_std_17)
ament_target_dependencies(manual_control_node
  drone_interfaces rclcpp sensor_msgs)

add_executable(px4_gateway_node src/px4_gateway_node.cpp)
target_compile_features(px4_gateway_node PRIVATE cxx_std_17)
ament_target_dependencies(px4_gateway_node
  drone_interfaces px4_msgs rclcpp)

add_executable(vehicle_tf_broadcaster_node src/vehicle_tf_broadcaster_node.cpp)
target_compile_features(vehicle_tf_broadcaster_node PRIVATE cxx_std_17)
ament_target_dependencies(vehicle_tf_broadcaster_node
  geometry_msgs nav_msgs px4_msgs rclcpp tf2 tf2_ros)

install(TARGETS
  automated_mission_node
  flight_supervisor_node
  gimbal_control_node
  manual_control_node
  px4_gateway_node
  vehicle_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
