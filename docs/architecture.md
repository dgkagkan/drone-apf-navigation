# Drone APF Architecture

The active controller is split into small ROS 2 nodes. Only one node is allowed
to publish commands to PX4.

## Data Flow

```text
/joy -> manual_control -----------\
                                    command_mux -> apf_safety -> px4_gateway -> /fmu/in/*
/navigate_to -> navigation_server /
automated_mission ----------------/

/scan_3d/points -> lidar_processor -> /perception/obstacles -> apf_safety
/fmu/out/* -> px4_gateway -> /vehicle/state -> control and navigation nodes
```

## Packages

- `drone_interfaces`: shared messages, services, and actions.
- `drone_control`: joystick intent, gimbal control, flight supervision,
  automated mission sequencing, and the PX4 gateway.
- `drone_navigation`: LiDAR filtering, command arbitration, shared 3D APF,
  goal validation/action handling, and passive RViz visualization.
- `drone_bringup`: launch composition and simulation configuration.
- `apf_optuna`: trial orchestration and scoring using the same modular APF path.

## Ownership

- `px4_gateway_node` is the sole owner of `/fmu/in/offboard_control_mode`,
  `/fmu/in/trajectory_setpoint`, and `/fmu/in/vehicle_command`.
- `flight_supervisor_node` owns arm, disarm, takeoff, landing, and VTOL
  transition operations. It sends internal requests to the PX4 gateway.
- `command_mux_node` selects supervisor, manual, or autonomous intent in that
  priority order. Holding L1 marks manual input as an override.
- `apf_safety_node` applies the shared 3D APF to whichever intent is selected.
- `navigation_server_node` validates geofence limits before accepting a
  `/navigate_to` action. Manual override pauses the active goal and releasing
  L1 resumes it from the current vehicle position. A goal completes inside its
  configured 25m horizontal radius. The next queued goal continues directly;
  when no next goal arrives, the VTOL transitions to MC and holds the altitude
  captured at arrival.
- `navigation_client_node` provides the interactive terminal, validates and
  queues multiple goals, and sends them sequentially to `/navigate_to`.
- `apf_visualizer_node` receives each active navigation goal and draws a
  kinematically feasible nominal path without APF from the vehicle's current
  heading. The path respects configured FW turn-rate and pitch limits. It also
  publishes the independent trail of positions the vehicle actually flew.
- `automated_mission_node` owns only the automated mission state machine and
  Optuna telemetry. It does not publish PX4 messages or calculate APF forces.
- `gimbal_control_node` owns the simulated camera pan and tilt targets.

## Launch Files

- `controller.launch.py`: manual PS4 control, navigation action, APF, gimbal,
  and optional RViz visualization.
- `automated_controller.launch.py`: automated/Optuna mission using the same
  LiDAR, APF, supervisor, and gateway nodes.
- `apf.launch.py`: compatibility wrapper for the modular automated mission.

## Main APIs

```bash
ros2 action send_goal /navigate_to drone_interfaces/action/NavigateTo \
  "{target: {header: {frame_id: map}, pose: {position: {x: 300.0, y: 0.0, z: 15.0}}}, cruise_speed_m_s: 20.0, use_fixed_wing: true}"

ros2 action send_goal /takeoff drone_interfaces/action/Takeoff \
  "{target_altitude_m: 15.0, climb_speed_m_s: 3.0}"

ros2 service call /flight/arm drone_interfaces/srv/Arm "{arm: true}"
```

## Interactive Navigation Terminal

`controller.launch.py` opens `navigation_client_node` in a separate terminal by
default. The terminal remains responsive while an action is running and stores
approved goals in FIFO order. Live feedback is refreshed in a fixed dashboard
area, so it does not erase or interrupt the command currently being typed.

```text
give command> arm
give command> takeoff 15
give command> goal
give coordinates (x y z)> 300 0 15
give command> goal 500 -20 20
```

Available commands are `arm`, `takeoff [altitude] [climb_speed]`, `goal [x y z]`,
`speed <m/s>`, `status`, `queue`, `cancel`, `clear`, `help`, and `quit`. Takeoff
defaults to 15m at 2m/s, and queued navigation goals wait for it to complete.
Every goal is checked before it enters the queue. The terminal prints `APPROVED`
or `REJECTED` with the reason and reports live ENU position, velocity, total
speed, remaining distance, and APF state.

The navigation geofence is a 2000x2000x2000m cube centered at the first valid
vehicle position. Targets below the configured minimum flight altitude are also
rejected. Disable the popup with `navigation_client_terminal:=false`.

The PS4 D-pad controls the camera: left/right changes pan and up/down changes
tilt. The rates and limits are in `drone_control/config/controller.yaml`.

Manual flight is enabled while L1 is held. L1+L2 requests arm and Offboard,
L1+Circle disarms, L1+Triangle requests fixed-wing mode, and L1+Square requests
multicopter mode. Every manual motion command still passes through APF safety.
