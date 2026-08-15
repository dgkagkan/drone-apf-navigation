cmake_minimum_required(VERSION 3.8)
project(drone_description)

option(
  DRONE_ONBOARD_BUILD
  "Install only the URDF needed by drone_brain.launch.py"
  OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(DRONE_ONBOARD_BUILD)
  install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
else()
  install(DIRECTORY urdf rviz models DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
