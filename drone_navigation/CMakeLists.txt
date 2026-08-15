cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(apf_solver src/apf_solver.cpp)
target_compile_features(apf_solver PUBLIC cxx_std_17)
target_include_directories(apf_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(nominal_path_planner src/nominal_path_planner.cpp)
target_compile_features(nominal_path_planner PUBLIC cxx_std_17)
target_include_directories(nominal_path_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(apf_safety_node src/apf_safety_node.cpp)
target_compile_features(apf_safety_node PRIVATE cxx_std_17)
target_link_libraries(apf_safety_node apf_solver)
ament_target_dependencies(apf_safety_node
  drone_interfaces rclcpp sensor_msgs std_srvs)

add_executable(apf_visualizer_node src/apf_visualizer_node.cpp)
target_compile_features(apf_visualizer_node PRIVATE cxx_std_17)
target_link_libraries(apf_visualizer_node nominal_path_planner)
ament_target_dependencies(apf_visualizer_node
  drone_interfaces geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs)

add_executable(command_mux_node src/command_mux_node.cpp)
target_compile_features(command_mux_node PRIVATE cxx_std_17)
ament_target_dependencies(command_mux_node
  drone_interfaces rclcpp std_msgs)

add_executable(lidar_processor_node src/lidar_processor_node.cpp)
target_compile_features(lidar_processor_node PRIVATE cxx_std_17)
ament_target_dependencies(lidar_processor_node
  geometry_msgs rclcpp sensor_msgs tf2 tf2_ros)

add_executable(point_cloud_throttle_node src/point_cloud_throttle_node.cpp)
target_compile_features(point_cloud_throttle_node PRIVATE cxx_std_17)
ament_target_dependencies(point_cloud_throttle_node rclcpp sensor_msgs)

add_executable(route_executor_node src/route_executor_node.cpp)
target_compile_features(route_executor_node PRIVATE cxx_std_17)
ament_target_dependencies(route_executor_node
  drone_interfaces rclcpp rclcpp_action)

add_executable(navigation_server_node src/navigation_server_node.cpp)
target_compile_features(navigation_server_node PRIVATE cxx_std_17)
ament_target_dependencies(navigation_server_node
  drone_interfaces geometry_msgs rclcpp rclcpp_action std_msgs)

add_executable(navigation_client_node src/navigation_client_node.cpp)
target_compile_features(navigation_client_node PRIVATE cxx_std_17)
ament_target_dependencies(navigation_client_node
  drone_interfaces rclcpp rclcpp_action std_srvs)

install(TARGETS
  apf_solver
  nominal_path_planner
  apf_safety_node
  apf_visualizer_node
  command_mux_node
  lidar_processor_node
  point_cloud_throttle_node
  route_executor_node
  navigation_client_node
  navigation_server_node
  EXPORT export_apf_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(export_apf_solver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_apf_solver test/test_apf_solver.cpp)
  target_compile_features(test_apf_solver PRIVATE cxx_std_17)
  target_link_libraries(test_apf_solver apf_solver)
  ament_add_gtest(test_nominal_path_planner test/test_nominal_path_planner.cpp)
  target_compile_features(test_nominal_path_planner PRIVATE cxx_std_17)
  target_link_libraries(test_nominal_path_planner nominal_path_planner)
endif()

ament_package()
