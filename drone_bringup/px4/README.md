# PX4 reference files

These are **version-controlled copies** of files that must live inside the
PX4-Autopilot tree to take effect. The copies here are for reference / backup
so the project is self-documenting; the **active** files are the ones inside
`~/drone_project/PX4-Autopilot`.

## `4030_gz_standard_vtol_lidar`

Custom PX4 SITL airframe that spawns the `standard_vtol_lidar` gz model
(base `standard_vtol` + a 360° 2D `gpu_lidar` publishing on gz topic `scan`).
It is a copy of `4004_gz_standard_vtol` plus the APF arming block
(`NAV_DLL_ACT=0`, `SYS_HAS_NUM_ASPD=0`, `COM_RCL_EXCEPT=4`) and
`PX4_SIM_MODEL=standard_vtol_lidar`.

### Why it's needed
The stock `make px4_sitl gz_standard_vtol` target hard-sets
`PX4_SIM_MODEL=gz_standard_vtol`, so `px4-rc.gzsim` spawns the base
`standard_vtol` model — which has **no lidar**. A dedicated airframe named
`*_gz_standard_vtol_lidar` makes PX4 spawn the lidar model instead (mirrors
`4013_gz_x500_lidar_2d`).

### Install into PX4 (already done on this machine)
1. Copy this file to
   `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_standard_vtol_lidar`
2. Add `4030_gz_standard_vtol_lidar` to
   `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
3. Rebuild: `make px4_sitl`

### Run
```bash
cd ~/drone_project/PX4-Autopilot
make px4_sitl gz_standard_vtol_lidar_forest   # forest world
# or gz_standard_vtol_lidar for the default world
```
The gz model itself is copied under
`drone_description/models/standard_vtol_lidar/` (it depends on PX4's base
`standard_vtol` model being on the gz resource path).
