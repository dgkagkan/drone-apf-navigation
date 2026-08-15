cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

option(
  DRONE_ONBOARD_BUILD
  "Keep the complete shared interface set in the onboard profile"
  OFF)

# PC and onboard computers generate the same interfaces so their wire-level
# APIs remain identical. Declaring the shared profile flag here also keeps a
# workspace-wide -DDRONE_ONBOARD_BUILD argument warning-free.

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ApfTelemetry.msg"
  "msg/AutomatedMissionTelemetry.msg"
  "msg/FlightRequest.msg"
  "msg/MotionCommand.msg"
  "msg/RouteTarget.msg"
  "msg/SwarmDroneHeartbeat.msg"
  "msg/SwarmDroneState.msg"
  "msg/SwarmMissionCommand.msg"
  "msg/SwarmMissionFeedback.msg"
  "msg/SwarmRoute.msg"
  "msg/VehicleState.msg"
  "msg/SwarmAssignment.msg"
  "msg/SwarmState.msg"
  "srv/Arm.srv"
  "srv/SetApfMode.srv"
  "srv/ValidateGoal.srv"
  "srv/AddSwarmTarget.srv"
  "srv/RegisterSwarmDrone.srv"
  "srv/RemoveSwarmTarget.srv"
  "srv/SwarmCommand.srv"
  "action/Land.action"
  "action/NavigateTo.action"
  "action/Takeoff.action"
  "action/TransitionVtol.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
