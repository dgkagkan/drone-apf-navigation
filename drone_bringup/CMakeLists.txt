cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

option(
  DRONE_ONBOARD_BUILD
  "Install only launch and DDS configuration used by drone_brain.launch.py"
  OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(DRONE_ONBOARD_BUILD)
  install(FILES
    launch/controller.launch.py
    launch/drone_brain.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
  install(FILES
    config/fastdds_ethernet_pi.xml
    config/fastdds_loopback.xml
    DESTINATION share/${PROJECT_NAME}/config)
else()
  install(DIRECTORY
    launch
    config
    worlds
    px4
    media
    DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
