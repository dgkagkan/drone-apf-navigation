cmake_minimum_required(VERSION 3.8)
project(drone_swarm)

option(
  DRONE_ONBOARD_BUILD
  "Build only swarm_member_node for an onboard drone computer"
  OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(swarm_member_node src/swarm_member_node.cpp)
target_compile_features(swarm_member_node PRIVATE cxx_std_17)
ament_target_dependencies(swarm_member_node
  drone_interfaces geometry_msgs rclcpp rclcpp_action)

install(TARGETS
  swarm_member_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(NOT DRONE_ONBOARD_BUILD)
  find_package(nav_msgs REQUIRED)
  find_package(octomap REQUIRED)
  find_package(octomap_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  add_library(global_map_fusion src/global_map_fusion.cpp)
  target_compile_features(global_map_fusion PUBLIC cxx_std_17)
  target_include_directories(global_map_fusion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  add_library(assignment_solver src/assignment_solver.cpp)
  target_compile_features(assignment_solver PUBLIC cxx_std_17)
  target_include_directories(assignment_solver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  add_library(route_solver src/route_solver.cpp)
  target_compile_features(route_solver PUBLIC cxx_std_17)
  target_include_directories(route_solver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  add_library(flight_cost_model src/flight_cost_model.cpp)
  target_compile_features(flight_cost_model PUBLIC cxx_std_17)
  target_include_directories(flight_cost_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  add_executable(swarm_coordinator_node src/swarm_coordinator_node.cpp)
  target_compile_features(swarm_coordinator_node PRIVATE cxx_std_17)
  target_link_libraries(swarm_coordinator_node flight_cost_model route_solver)
  ament_target_dependencies(swarm_coordinator_node
    drone_interfaces geometry_msgs rclcpp rclcpp_action)

  add_executable(swarm_gimbal_router_node src/swarm_gimbal_router_node.cpp)
  target_compile_features(swarm_gimbal_router_node PRIVATE cxx_std_17)
  ament_target_dependencies(swarm_gimbal_router_node
    drone_interfaces rclcpp sensor_msgs std_msgs)

  add_executable(swarm_operator_node src/swarm_operator_node.cpp)
  target_compile_features(swarm_operator_node PRIVATE cxx_std_17)
  ament_target_dependencies(swarm_operator_node drone_interfaces rclcpp rclcpp_action)

  add_executable(swarm_visualization_node src/swarm_visualization_node.cpp)
  target_compile_features(swarm_visualization_node PRIVATE cxx_std_17)
  ament_target_dependencies(swarm_visualization_node
    drone_interfaces nav_msgs rclcpp visualization_msgs)

  add_executable(swarm_global_map_server src/swarm_global_map_server.cpp)
  target_compile_features(swarm_global_map_server PRIVATE cxx_std_17)
  target_link_libraries(swarm_global_map_server global_map_fusion ${OCTOMAP_LIBRARIES})
  target_include_directories(swarm_global_map_server PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  ament_target_dependencies(swarm_global_map_server
    drone_interfaces octomap_msgs rclcpp sensor_msgs visualization_msgs)

  install(TARGETS
    assignment_solver
    flight_cost_model
    route_solver
    global_map_fusion
    swarm_coordinator_node
    swarm_gimbal_router_node
    swarm_operator_node
    swarm_visualization_node
    swarm_global_map_server
    EXPORT export_assignment_solver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

  ament_export_targets(export_assignment_solver HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_assignment_solver test/test_assignment_solver.cpp)
    target_compile_features(test_assignment_solver PRIVATE cxx_std_17)
    target_link_libraries(test_assignment_solver assignment_solver)
    ament_add_gtest(test_route_solver test/test_route_solver.cpp)
    target_compile_features(test_route_solver PRIVATE cxx_std_17)
    target_link_libraries(test_route_solver route_solver)
    ament_add_gtest(test_flight_cost_model test/test_flight_cost_model.cpp)
    target_compile_features(test_flight_cost_model PRIVATE cxx_std_17)
    target_link_libraries(test_flight_cost_model flight_cost_model)
    ament_add_gtest(test_global_map_fusion test/test_global_map_fusion.cpp)
    target_compile_features(test_global_map_fusion PRIVATE cxx_std_17)
    target_link_libraries(test_global_map_fusion global_map_fusion)
  endif()
endif()

ament_package()
