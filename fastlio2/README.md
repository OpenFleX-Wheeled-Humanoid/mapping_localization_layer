# fastlio2

English | [中文](./README-CN.md)

---

`fastlio2` is the lightweight FAST-LIO2 LiDAR-inertial odometry package for the OpenFlex chassis. It consumes Livox MID360 point cloud and IMU data, estimates LiDAR/body odometry, and publishes odometry, local clouds, path, and optional TF.

## Package Role

Use this package when you want a direct FAST-LIO2 odometry node without the FAST-LIVO compatibility bridge.

Main executable:

```bash
lio_node
```

Main launch file:

```bash
launch/lio_launch.py
```

## Inputs

| Topic | Type | Description |
|---|---|---|
| `/livox/lidar` | `livox_interfaces2/msg/CustomMsg` | MID360 point cloud |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 built-in IMU |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry, used by mapping mode for static detection and map->odom computation |
| `/pgo/offset` | `nav_msgs/msg/Odometry` | PGO loop-closure correction, used only when map->odom mode is enabled |

## Outputs

The launch runs the node under namespace `fastlio2`, so relative outputs appear under `/fastlio2`.

| Topic | Type | Description |
|---|---|---|
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | LIO odometry |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | Current scan transformed into body frame |
| `/fastlio2/world_cloud` | `sensor_msgs/msg/PointCloud2` | Current scan transformed into world/map frame |
| `/fastlio2/lio_path` | `nav_msgs/msg/Path` | Estimated trajectory |

Depending on configuration, the node can also publish TF:

| TF | Mode |
|---|---|
| `odom -> base_link` | navigation-style odometry when `publish_tf` is true |
| `map -> odom` | mapping mode when `publish_map_odom` is true |

## Configuration Files

| File | Intended Use |
|---|---|
| `config/lio.yaml` | Default standalone LIO configuration |
| `config/lio_navigation.yaml` | Navigation configuration, `world_frame=odom`, `publish_tf=false` |
| `config/lio_navigation_gps.yaml` | Navigation with external EKF/GPS TF ownership |
| `config/lio_mapping.yaml` | Mapping configuration, `world_frame=map`, publishes `map -> odom` with PGO offset support |

Important parameters:

| Parameter | Meaning |
|---|---|
| `imu_topic` / `lidar_topic` | Input MID360 topics |
| `body_frame` | Output body frame, usually `base_link` or `mid360_link` |
| `world_frame` | `odom` for navigation, `map` for mapping |
| `publish_tf` | Whether to publish body TF |
| `publish_map_odom` | Whether to publish `map -> odom` |
| `r_ib` / `t_ib` | IMU body to `base_link` extrinsic |
| `r_il` / `t_il` | LiDAR to IMU extrinsic |

## Prerequisites

Start the MID360 driver first:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then verify:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## Start

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch fastlio2 lio_launch.py
```

`lio_launch.py` loads `config/lio.yaml` and starts RViz with `rviz/fastlio2.rviz`.

## Run Node Directly

```bash
ros2 run fastlio2 lio_node --ros-args \
  -p config_path:=~/openflex_all/openflex_ws/src/openflex_chassis/mapping_localization_layer/fastlio2/config/lio_navigation.yaml
```

## Check Output

```bash
ros2 topic list | grep fastlio2
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
ros2 topic echo --once /fastlio2/lio_odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Notes

- Livox MID360 IMU acceleration from `livox_ros_driver2` is in `g`; this node applies an internal `imu_accel_scale` when configured.
- In navigation systems where another node owns `odom -> base_link`, keep `publish_tf: false`.
- In mapping mode, use the mapping launch in higher-level bringup or `fast_livo`/`pgo` integration if loop closure is needed.
