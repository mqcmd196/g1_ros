# g1_navigation

Nav2 localization + navigation for the Unitree G1 against a **2D occupancy grid**
map, driving the robot through `/cmd_vel` (the gear_sonic locomotion bridge).

## TF / architecture

```
map ──AMCL──> odom ──FAST-LIO2──> base_link ──static──> pelvis ──> (robot TF: torso, livox_frame, legs…)
 (2D scan match)     (LIO odometry)            (base_link->pelvis)
/livox/lidar ─pointcloud_to_laserscan─> /scan  (AMCL + costmap obstacle layer)
nav2 (costmaps[static=2D map, obstacle=/scan] + planner + RPP controller + bt) ──> /cmd_vel ──> gear_sonic
```

- **Odometry**: FAST-LIO2 (Livox mid360 + IMU) publishes `odom -> base_link`.
- **Global localization**: AMCL matches `/scan` against the 2D map, publishes `map -> odom`.
- **Map**: not bundled (site-specific). Pass a map_server-format 2D occupancy grid
  via `map:=/path/to/map.yaml` (required).

## Assumptions to check

- **Torso/waist held fixed** during navigation (mid360 is torso-mounted). Set the
  static `base_link -> pelvis` from the neutral pose:
  `ros2 run tf2_ros tf2_echo imu_in_torso pelvis`, then pass
  `base_to_pelvis_{x,y,z,yaw,pitch,roll}:=…` to `fast_lio.launch.py`.
- `pelvis` has roll/pitch/height while walking; nav2 uses `base_link` (x,y,yaw). Add a
  ground-projected `base_footprint` if 2D accuracy suffers.
- If the 2D scan projection is too noisy, switch to 3D `.pcd` localization (NDT/GLIM).

## Run

```bash
ros2 launch g1_navigation g1_navigation.launch.py map:=/path/to/map.yaml
ros2 launch g1_navigation g1_navigation.launch.py map:=/path/to/map.yaml use_fast_lio:=false  # external odom
```

Send goals from RViz2 (Nav2 panel) or the `navigate_to_pose` action.

## Deps

`nav2_*`, `pointcloud_to_laserscan` come from apt (rosdep). `fast_lio` and
`livox_ros_driver2` are source deps — see the workspace `jazzy.repos.yaml`
(and `.github/upstream.jazzy.repos` for CI).
