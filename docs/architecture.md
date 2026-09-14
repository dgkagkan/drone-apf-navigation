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
/fmu/out/* (including BatteryStatus) -> px4_gateway -> /vehicle/state
    -> onboard swarm_member -> /swarm/drone_heartbeat -> swarm_coordinator
    -> /swarm/state -> dashboard

raw LiDAR -> point_cloud_throttle (per drone, PC or onboard)
    -> /swarm/<drone_id>/map_cloud -> swarm_global_map_server -> global map/RViz
```

`lidar_processor` keeps the available horizontal LiDAR field of view and
publishes map-frame obstacle points. `apf_safety` selects relevant points from
the shortest angular sector between measured horizontal velocity and the
unmodified selected intent. The sector receives a speed-dependent margin, and
an omnidirectional emergency radius bypasses angular rejection. The existing
APF force equation then consumes only those selected points.

Sector filtering is configured with `sector_margin_min_deg`,
`sector_margin_max_deg`, `sector_margin_speed_min`,
`sector_margin_speed_max`, `sector_direction_min_speed`, and
`emergency_radius`. Its exact runtime result is available in `/apf/telemetry`.
The `/apf/obstacles_used` and `/apf/obstacles_sector_ignored` clouds show which
points passed or failed the angular filter.

For swarm visualization, each registered LiDAR drone publishes a typed mapping
cloud on `/swarm/<drone_id>/map_cloud`. The existing throttle node transforms
the cloud into `map` using TF at the scan timestamp and includes the LiDAR
origin plus the configured maximum range. Its existing per-drone filtered cloud
continues to feed APF unchanged. Max-range rays are kept only in the mapping
message so they clear free space without becoming APF obstacles.

`swarm_global_map_server` subscribes to `/swarm/state` and creates/removes
mapping subscriptions dynamically. It maintains **one complete local OctoMap
per LiDAR drone and one global occupied map**, for any number of drones. Local
trees retain their own probabilistic occupied/free-space evidence. The global
map is not a Boolean union of local occupied cells. Every scan contributes its
raw hit endpoints and miss rays directly, so verified free space from any drone
can clear a voxel that was previously seen occupied by any other drone.

Global fusion and each local occupied cache use the same evidence policy.
By default (`dynamic_obstacle_timeout_sec=0`), hits immediately enter the
persistent layer; they do not expire merely because they are no longer seen.
Miss rays reduce log-odds instead of instantly hiding a voxel. This retains
sparse floor hits and avoids toggling local/global colors on isolated misses.
Enough newer misses still clear any obstacle, regardless of which drone first
observed it. With default probabilities, a saturated voxel clears after four
misses without intervening hits. There is no object-specific floor exemption.

A positive `dynamic_obstacle_timeout_sec` opts into the previous temporal mode:
recent hits expire when unseen unless `static_confirmation_sec` and
`static_confirmation_hits` promote them to persistent geometry. Free rays clear
temporal occupancy immediately; persistent geometry needs probabilistic misses.
This mode trades retention of sparse surfaces for removal of unobserved moving
objects and may cause local-color flicker. Same-time hit/free conflicts in the
temporal layer favor the hit. Both modes publish one composed global map.

Each drone has its own insertion worker. The standard OctoMap batch ray update
produces the free/hit key sets once; those sets update both its full local tree
and the global fusion. Only occupied/free transitions are cached for
publication, and temporal expiry uses a per-voxel deadline queue rather than a
full-map scan. Scan timestamps prevent repeated processing of the same
nonzero-stamped scan. Session/reset generations prevent in-flight scans from
restoring a cleared local contribution. Reset/disconnect policy affects that
drone's local diagnostic map only; the source-neutral global map is corrected
by later free rays or temporal expiry. Retained local maps survive a temporary
disconnect/reconnect until `submap_timeout_sec`.

The RViz cloud publisher and binary/full serializer have separate workers with
coalesced requests (no growing queue). They copy only occupied keys or pending
global changes under the shared lock; encoding, ray insertion and DDS publishing
run outside it. The global serialized tree is updated incrementally. Identical
geometry is not regenerated on every timer tick. Local/debug clouds and
serialization are generated on demand, including when a subscriber joins an
otherwise idle map. Default launch rates remain 5 Hz for changed RViz geometry
and 1 Hz for changed binary/full maps; these are ceilings, not guaranteed scan
throughput. The mapper logs snapshot/encode/publish time for profiling.

The fused occupied centers are published on
`/swarm/octomap_point_cloud_centers` and the binary/full OctoMap messages on
`/swarm/octomap_binary` and `/swarm/octomap_full`. Per-drone occupied contributions are available on
`/swarm/mapping/<drone_id>/occupied_voxels`; these topics are created as drones
join, so no drone IDs are hardcoded in the mapper.
The default RViz display is `Global OctoMap only (height)`, using
`/swarm/octomap_point_cloud_centers` to show the complete global occupied
centers with height colors. The optional `/swarm/mapping_visualization` display
uses exactly the same global geometry but lets each connected local map replace
the color of matching global voxels. Local colors come from a registration index
and a hue sequence excluding yellow/green.
Indices survive reconnects within the mapper process; restarting with a different
registration order can change them. When several locals cover a voxel, the first
drone ID in lexical order wins. This single point cloud avoids coincident geometry
and never restores a globally cleared voxel from stale local evidence.
Local maps include retained history, not a rolling Nav2 window. The optional
`Global OctoMap only (height)` display shows the complete height-colored global map;
disable the combined display when using it. The separate `Local map contributions`
diagnostic display remains disabled by default. Resolution (0.5 m), point size,
and sensor range (300 m) are unchanged.

The combined cloud's base palette runs blue–cyan–green–yellow–red from low to
high Z, clamped to the mapper's `visualization_min_z_m` (-1) and
`visualization_max_z_m` (60). Fixed bounds avoid whole-map recoloring when a new
height extreme appears. The standalone RViz AxisColor display has separate
height bounds, initially matching these defaults. Local colors still take priority.

The main mapping LiDAR has a 360-degree horizontal sweep but only ±15-degree
vertical coverage. Its downward blind region is not a ground filter: no floor
voxels can be inserted there without actual returns. The separate downward
landing sensor is currently not an input to the mapper.

## Packages

- `drone_interfaces`: shared messages, services, and actions.
- `drone_control`: joystick intent, gimbal control, flight supervision,
  automated mission sequencing, and the PX4 gateway.
- `drone_navigation`: LiDAR filtering, command arbitration, shared 3D APF,
  goal validation/action handling, and passive RViz visualization.
- `drone_swarm`: buffered target submission, dynamic healthy-drone snapshots,
  multi-drone route optimization, asynchronous per-drone route dispatch, and
  dynamic per-drone global map fusion.
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
- `point_cloud_throttle_node` owns only the low-rate mapping relay. When
  `mapping_enabled:=true` on `drone_brain.launch.py`, it can run on the
  onboard computer; it does not build an OctoMap or touch PX4.
- `swarm_global_map_server` is PC-only. It discovers registered/connected
  LiDAR drones from `/swarm/state`, subscribes to their typed map clouds, and
  owns all global mapping evidence and removal policy.

## Launch Files

- `controller.launch.py`: manual PS4 control, navigation action, APF, gimbal,
  and optional RViz visualization.
- `swarm_sim.launch.py`: a configurable number of independent PX4 SITL
  vehicles, one XRCE agent and namespaced controller stack per vehicle, plus
  the swarm coordinator in one Gazebo world.
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
validated and stored in `pending_targets`. `CALCULATE` or `RECALCULATE` only
snapshots the targets and healthy drones, builds ordered routes, stores them as
`planned_routes`, and publishes `planned_assignments` in `/swarm/state` for map
preview. No drone receives a route at this stage. The separate typed
`START_MISSION` command revalidates that the target set and selected drones have
not changed and then publishes one broadcast containing all routes on
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
best balanced insertion, and improves the result with 2-opt and cross-route
relocation passes. The route objective is lexicographic: minimize the highest
drone completion cost first, then the spread between the longest and shortest
routes, and finally the total swarm cost. Every target appears exactly once.
Costs are estimated independently for each drone in seconds from commanded
speed, vertical travel, current heading, fixed-wing turn rate, VTOL transitions,
and configurable APF detour factors. Workload and route-change penalties avoid
unnecessary reassignment. Battery penalties and time/energy reserves reject
routes that cannot be completed safely while preserving return-home reserve.

Battery safety is decided locally by `swarm_member_node`, so it remains active
if the coordinator or Ethernet link disappears. A critical battery latches an
RTH request and stops the drone accepting new tasks. The coordinator excludes
that drone, returns only its unfinished targets to `pending_targets`, sends a
targeted home route, and recalculates those tasks over the remaining healthy
drones. If no coordinator acknowledgement arrives before the configured
timeout, the local brain sends its own home goal. Emergency battery causes a
local LAND. The latch clears only while disarmed and above the recovery level;
manual OFF/REJOIN remains a separate operator state.

```bash
ros2 service call /swarm/add_target drone_interfaces/srv/AddSwarmTarget \
  "{target: {header: {frame_id: map}, pose: {position: {x: 25.0, y: 700.0, z: 14.0}}}, cruise_speed_m_s: 20.0, use_fixed_wing: true}"

# CALCULATE=0, RECALCULATE=3, START_MISSION=13
ros2 service call /swarm/command drone_interfaces/srv/SwarmCommand "{command: 0}"
ros2 service call /swarm/command drone_interfaces/srv/SwarmCommand "{command: 13}"
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
swarm> start
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
to add a target with the selected altitude, speed, and VTOL mode. `CALCULATE`
and `RECALCULATE` draw a route preview without moving a drone; `START MISSION`
dispatches the displayed plan. The dashboard also supports target removal,
clear pending,
mission cancel, swarm or per-drone arm/takeoff/home, dynamic drone cards, route
progress, nominal/flown paths, and APF vectors.

Each target stores its own `CRUISE SPEED (m/s)` and passes it through the
coordinator route to the existing `NavigateTo` action. Each drone card also has
an optional live speed override. The coordinator broadcasts that typed command,
every route executor filters it by `drone_id`, and the selected drone updates its
local navigation command without restarting the route. `AUTO` clears the
override and returns to each route target's stored speed. Drone cards and map
labels show the measured 3D speed from the namespaced `VehicleState` velocity.
Each map icon also uses the PX4 body heading carried through
`VehicleState -> coordinator -> SwarmState`, so its nose points where the drone
is facing. The dashboard never publishes a direct PX4 velocity command.

`HOME ALL`, per-drone `HOME`, per-drone `LAND`, and per-drone `OFF` carry an
explicit `target_drone_ids` list. Each route executor ignores commands that do
not contain its own ID. Only the selected drone's route is detached, its
unfinished targets return to the pending buffer, and an auxiliary home route
can run without cancelling other drones. HOME returns to the exact recorded
spawn x/y at safe altitude, switches to multicopter for the configured precision
approach, and then holds there. A normal operator HOME never commands LAND;
only the separate low-battery RTH route lands automatically after reaching
home.

Gazebo camera images are bridged only on the simulation PC. The simulated
camera is configured as 640x360 at 10 Hz. The dashboard request and encoding
loop is configurable up to 60 Hz, but preview FPS remains limited by the camera
source. DDS sends image samples only to matched subscribers, so a remote
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
  network_mode:=lan ros_domain_id:=0

ros2 launch drone_bringup add_sim_vehicle.launch.py \
  drone_id:=drone_4 px4_instance:=3 system_id:=4 agent_port:=8891 \
  network_mode:=lan ros_domain_id:=0

# Raspberry Pi
ros2 launch drone_bringup drone_brain.launch.py \
  drone_id:=drone_4 target_system:=4 use_sim_time:=true \
  use_ground_truth:=false lidar_points_topic:=scan_3d/filtered_points \
  publish_robot_description:=false network_mode:=lan ros_domain_id:=0
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

## Configurable swarm simulation

The complete local swarm simulation uses one custom VTOL model per vehicle and
keeps each autopilot path independent. The default is three vehicles, but the
same launch creates `drone_1` through `drone_N`:

| Drone | PX4 instance | MAV system ID | XRCE UDP port | Initial map position |
| --- | ---: | ---: | ---: | --- |
| `drone_1` | 0 | 1 | 8888 | `(0, 0, 0)` |
| `drone_2` | 1 | 2 | 8889 | `(0, 8, 0)` |
| `drone_3` | 2 | 3 | 8890 | `(0, -8, 0)` |

```bash
colcon build --packages-select drone_interfaces drone_description \
  drone_control drone_navigation drone_swarm drone_dashboard drone_bringup
source install/setup.bash
ros2 launch drone_bringup swarm_sim.launch.py drones:=3

# For example, start five complete simulated vehicles.
ros2 launch drone_bringup swarm_sim.launch.py drones:=5
```

For each generated ID the launch allocates a unique PX4 instance, MAV system
ID, XRCE UDP port, DDS namespace, Gazebo model/topic set, controller and TF
stack, mapping relay and RViz displays. The first three positions stay
backwards-compatible with the original layout. Later vehicles are placed on
expanding six-point rings around the origin; `drone_spawn_spacing_m` controls
the ring spacing and `base_agent_port` controls the first XRCE port. The
generated bridge and swarm parameter files are written to
`work_root/generated` for that run, so the checked-in three-drone YAML files
remain unchanged.

The supported configuration range is `drones:=0` through `drones:=255` in one
launch. The upper bound comes from PX4's one-byte MAV system ID and is a
practical finite bound, while CPU, GPU, memory, UDP ports and Gazebo determine
what a computer can run in practice. With `drones:=0`, only shared components
such as `/clock`, the coordinator and dashboard are started.

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
