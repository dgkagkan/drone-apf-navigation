# Drone APF Architecture

The active controller is split into small ROS 2 nodes. Only one node is allowed
to publish commands to PX4.

## Data Flow

```text
/joy -> manual_control -----------\
                                    command_mux -> apf_safety -> px4_gateway -> /fmu/in/*
/navigate_to -> navigation_server /
automated_mission ----------------/

/swarm/mission_command -> route_executor -> sequential /navigate_to goals
route_executor -> /swarm/mission_feedback -> swarm_coordinator

/scan_3d/points -> lidar_processor -> /perception/obstacles -> apf_safety
/fmu/out/* -> px4_gateway -> /vehicle/state -> control and navigation nodes
```

## Packages

- `drone_interfaces`: shared messages, services, and actions.
- `drone_control`: joystick intent, gimbal control, flight supervision,
  automated mission sequencing, and the PX4 gateway.
- `drone_navigation`: LiDAR filtering, command arbitration, shared 3D APF,
  goal validation/action handling, and passive RViz visualization.
- `drone_swarm`: buffered target submission, dynamic healthy-drone snapshots,
  multi-drone route optimization, and asynchronous per-drone route dispatch.
- `drone_dashboard`: PC-local HTTP control panel, mission map, fleet state,
  APF/path overlays, and compressed camera previews.
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
- `route_executor_node` is the headless local route memory used by the swarm.
  It receives the common `/swarm/mission_command` broadcast, extracts only the
  route matching its `drone_id`, stores it, and feeds the targets sequentially
  to the existing namespaced `/navigate_to` action. It publishes acknowledgements,
  progress, terminal results, and cancellation on `/swarm/mission_feedback`.
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
- `swarm_sim.launch.py`: three independent PX4 SITL vehicles, three XRCE
  agents, three namespaced controller stacks, and the swarm coordinator in one
  Gazebo world.
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

## Swarm Coordinator

The coordinator never moves a drone when a target is submitted. Each target is
validated and stored in `pending_targets`; routing starts only after a typed
`CALCULATE` or `RECALCULATE` command. At that moment the coordinator snapshots
the connected, localized, available drones and builds one ordered route per
selected drone. It then publishes one typed broadcast containing all routes on
`/swarm/mission_command`. Every drone receives the broadcast but processes only
the `SwarmRoute` whose `drone_id` matches its own ID. Targets move to
`active_targets` only after every selected drone acknowledges that it accepted
and stored its route through `/swarm/mission_feedback`.

The common command message carries `command_id`, `mission_id`, `revision`, an
`EXECUTE` or `CANCEL` command, and an array of typed per-drone routes. The common
feedback message carries those identifiers plus `drone_id`, `route_id`, state,
progress counters, remaining distance, and a diagnostic message. This preserves
action-like cancel and feedback semantics over a real one-to-many broadcast;
the existing per-drone `/navigate_to` interface remains a standard ROS 2 action.

Swarm membership is dynamic and lease-based. Every complete drone stack runs a
`swarm_member_node`, registers through `/swarm/register_drone`, and publishes a
typed heartbeat on `/swarm/drone_heartbeat`. Registration includes the unique
drone ID, namespace, boot-session ID, local API names, and capabilities. The
coordinator accepts a drone for routing only while its heartbeat, PX4 state,
localization, and navigation action are all fresh. A duplicate live drone ID is
rejected; a restarted drone can reclaim its ID after the previous lease expires.
The authoritative list and health of all members is published in
`/swarm/state.drones`, which also makes operator control dynamic.

The scalable route solver has no hard target-count limit. It starts from a
global drone-to-target seed assignment, adds every remaining target at its
lowest insertion cost, and improves the result with 2-opt and cross-route
relocation passes. Every target appears exactly once. The optimized cost is the
sum of the 3D legs from each current drone position through its ordered targets;
return-to-base is not required.

```bash
ros2 service call /swarm/add_target drone_interfaces/srv/AddSwarmTarget \
  "{target: {header: {frame_id: map}, pose: {position: {x: 25.0, y: 700.0, z: 14.0}}}, cruise_speed_m_s: 20.0, use_fixed_wing: true}"

# CALCULATE=0, CLEAR_PENDING_TARGETS=1, CANCEL_ACTIVE_MISSION=2, RECALCULATE=3
ros2 service call /swarm/command drone_interfaces/srv/SwarmCommand "{command: 0}"
```

The swarm terminal can prepare all drones or one selected drone before
assignment. `arm` and `takeoff` target all configured drones; adding a number
targets only that drone. Takeoff altitude and climb speed are configured in
`drone_swarm/config/swarm.yaml`.

```text
swarm> arm
swarm> arm 1
swarm> takeoff
swarm> takeoff 1
swarm> calculate
```

`drone_bringup/launch/swarm.launch.py` starts the coordinator and its terminal.
The drones must be in the same ROS domain and common `map` frame and must expose
namespaced `vehicle/state` and `navigate_to` APIs, plus a local route executor.
All route executors share `/swarm/mission_command` and `/swarm/mission_feedback`.
The coordinator also discovers newly appearing namespaced `vehicle/state` topics,
so each calculation can use a different healthy subset of the swarm.
Start each namespaced controller stack with `navigation_client_terminal:=false`
so the coordinator remains the only navigation-goal owner.

### Local web dashboard

`swarm_sim.launch.py` starts the dashboard by default and opens
`http://127.0.0.1:8765`. The browser never commands PX4 directly. Its typed HTTP
requests are translated to the existing ROS services/actions, while
`swarm_coordinator_node` remains the owner of pending targets, active targets,
routes, cancellation, and return-home state.

The map uses local ENU coordinates and works without internet map tiles. Click
to fill a target's east/north coordinates, choose altitude, speed, and VTOL
mode, then press `ADD TARGET`. Targets stay pending until `CALCULATE` or
`RECALCULATE`. The dashboard also supports one-target removal, clear pending,
mission cancel, swarm or per-drone arm/takeoff/home, dynamic drone cards, route
progress, nominal/flown paths, and APF vectors.

Each target stores its own `CRUISE SPEED (m/s)` and passes it unchanged through
the coordinator route to the existing `NavigateTo` action. Drone cards and map
labels show the measured 3D speed from the namespaced `VehicleState` velocity.
To change the requested speed safely, set it while adding the target and use
`CALCULATE`/`RECALCULATE`; the dashboard does not bypass the navigation server
with direct PX4 velocity commands.

`HOME ALL` or a per-drone `HOME` replaces the current coordinated mission.
Unfinished mission targets return to the pending buffer before the selected
return-home route is broadcast, so no old route continues outside coordinator
ownership.

Gazebo camera images are bridged only on the simulation PC. The simulated
camera is configured as 640x360 at 10 Hz, and the dashboard serves JPEG previews
at up to 5 Hz. DDS sends image samples only to matched subscribers, so a remote
Raspberry Pi brain does not receive camera traffic unless a camera subscriber
is deliberately started there.

Useful launch options:

```bash
# Default: dashboard enabled, browser opens automatically, terminal disabled.
ros2 launch drone_bringup swarm_sim.launch.py

# Do not open a browser (the dashboard server still runs).
ros2 launch drone_bringup swarm_sim.launch.py open_dashboard:=false

# Restore the old operator terminal alongside the dashboard.
ros2 launch drone_bringup swarm_sim.launch.py operator_terminal:=true

# Start only coordinator/dashboard, without Gazebo.
ros2 launch drone_bringup swarm.launch.py open_dashboard:=false
```

The dashboard binds to loopback by default. To view it from another trusted
machine on the direct Ethernet network, explicitly set `dashboard_host:=0.0.0.0`
in `swarm.launch.py` and open `http://<pc-ethernet-ip>:8765`. There is no login
layer, so it must not be exposed to an untrusted network.

`drone_bringup/launch/drone_brain.launch.py` starts one complete ROS brain
without Gazebo, PX4 SITL, RViz, or a coordinator. It is the launch used on a
Raspberry Pi or other onboard computer. In simulation,
`drone_bringup/launch/add_sim_vehicle.launch.py` runs on the main PC and adds
only the matching PX4 SITL instance, XRCE agent, Gazebo model, sensor bridges,
and throttled shared-map cloud. This keeps rendering on the PC while APF and
navigation execute on the independent drone computer.

For a remote brain connected directly over Ethernet, start the PC simulation
and the onboard brain in the same explicit ROS domain:

```bash
# Main PC
ros2 launch drone_bringup swarm_sim.launch.py \
  network_mode:=lan ros_domain_id:=10

ros2 launch drone_bringup add_sim_vehicle.launch.py \
  drone_id:=drone_4 px4_instance:=3 system_id:=4 agent_port:=8891 \
  network_mode:=lan ros_domain_id:=10

# Raspberry Pi
ros2 launch drone_bringup drone_brain.launch.py \
  drone_id:=drone_4 target_system:=4 use_sim_time:=true \
  use_ground_truth:=false lidar_points_topic:=scan_3d/filtered_points \
  publish_robot_description:=false network_mode:=lan ros_domain_id:=10
```

Gazebo Transport and the PX4-to-agent UDP links remain on the PC. ROS 2 uses the
direct Ethernet subnet for communication with the Pi. The throttled LiDAR topic
is used by the remote simulated brain instead of the full-rate raw cloud.
The LAN launch profiles explicitly allow only `lo` plus `enp12s0` on the PC and
`lo` plus `eth0` on the Pi, so DDS discovery and data cannot use the Wi-Fi
interface. If either Ethernet interface is renamed, update the matching
`fastdds_ethernet_pc.xml` or `fastdds_ethernet_pi.xml` file.

### Raspberry Pi onboard build

The normal build remains the full PC build. On a Raspberry Pi, use the dedicated
onboard profile so CMake omits the coordinator, operator, route solvers,
simulation cloud throttle, terminal navigation client, automated mission,
manual control, and gimbal control executables. APF visualization remains in the
onboard profile because the PC swarm visualizer aggregates the namespaced arrows
and paths produced by each drone.

```bash
cd ~/ros2_ws/src/drone-apf-navigation
./scripts/build_drone_brain.sh
source ~/ros2_ws/install/onboard/setup.bash
```

The script restricts package discovery to this repository, which avoids package
name collisions with stale package copies elsewhere in `src`. It uses separate
`build/onboard` and `install/onboard` directories and forces sequential,
single-job compilation to keep Raspberry Pi memory use bounded. The resulting
install contains the existing `drone_brain.launch.py`; its runtime ROS graph and
swarm protocol are unchanged. Manual control and gimbal control are disabled by
default for this headless brain but can still be enabled in a normal full build.
The small pure-Python dashboard package is included only to keep
`drone_bringup` dependency metadata complete; `drone_brain.launch.py` never
starts it and the Pi does not subscribe to camera images.

## Three-drone simulation

The complete local swarm simulation uses one custom VTOL model per vehicle and
keeps each autopilot path independent:

| Drone | PX4 instance | MAV system ID | XRCE UDP port | Initial map position |
| --- | ---: | ---: | ---: | --- |
| `drone_1` | 0 | 1 | 8888 | `(0, 0, 0)` |
| `drone_2` | 1 | 2 | 8889 | `(0, 8, 0)` |
| `drone_3` | 2 | 3 | 8890 | `(0, -8, 0)` |

```bash
colcon build --packages-select drone_interfaces drone_description \
  drone_control drone_navigation drone_swarm drone_dashboard drone_bringup
source install/setup.bash
ros2 launch drone_bringup swarm_sim.launch.py
```

The default swarm world is `test`, containing the tiled grass ground and the
90 distributed obstacles. Use `world:=optuna_course` when the optimization
course is needed.

Use `headless:=true use_rviz:=false open_dashboard:=false` for a non-GUI smoke test. The
launch forces ROS 2, Fast DDS, Gazebo Transport, and all PX4-to-agent links onto
the loopback interface. The physical Ethernet and Wi-Fi interfaces are not used.

Commands launched by `swarm_sim.launch.py` inherit the local-only environment.
For a separate terminal that must inspect the same graph, use:

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
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
