# Remote PS4 Controller Over Tailscale

The main PC runs PX4, Gazebo, and `controller`. The remote laptop runs only
`joy` and publishes `/joy` through Tailscale.

## Main PX4/Gazebo PC

```bash
source /opt/ros/jazzy/setup.bash
source /home/dimitris-gkagkanakis/ros2_work_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dimitris-gkagkanakis/ros2_work_ws/install/drone_bringup/share/drone_bringup/config/cyclonedds_main.xml
ros2 launch drone_bringup controller.launch.py use_local_joy:=false
```

## Remote Controller Laptop

Copy `cyclonedds_remote_joy.xml` to the remote laptop. With the PS4 controller
connected there, run:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds_remote_joy.xml
ros2 run joy game_controller_node --ros-args -p device_id:=0 -p autorepeat_rate:=30.0
```

## Changing The Remote Laptop

On the main PC, edit `cyclonedds_main.xml` and replace only this address with
the new controller laptop Tailscale IP:

```xml
<Peer Address="100.126.163.66" />
```

The main PC address is `100.65.195.88`. Change the peer in the remote laptop
file only if the main PC Tailscale IP changes.
