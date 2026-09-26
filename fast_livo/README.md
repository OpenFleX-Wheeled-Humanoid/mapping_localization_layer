# fast_livo

English | [中文](./README-CN.md)

---

`fast_livo` contains the FAST-LIVO2-based LiDAR/visual-inertial mapping node and an OpenFlex compatibility bridge. In the current chassis stack it is mainly used as a FAST-LIO-compatible pipeline for mapping and navigation.

## Package Role

Main executables:

| Executable | Role |
|---|---|
| `fastlivo_mapping` | FAST-LIVO2 core node; subscribes MID360 LiDAR/IMU and optional camera |
| `lio_bridge_node` | Converts raw FAST-LIVO2 outputs into OpenFlex `/fastlio2/*` topics and TF behavior |

Main launch files:

| Launch | Role |
|---|---|
| `launch/swerve_lio.launch.py` | OpenFlex LIO-compatible pipeline with core + bridge |
| `launch/mapping_mid360_realsense2.launch.py` | MID360 + RealSense visual-LiDAR mapping launch |

## Inputs

FAST-LIVO2 core:

| Topic | Type | Description |
|---|---|---|
| `/livox/lidar` | `livox_interfaces2/msg/CustomMsg` | MID360 point cloud |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 IMU |
| `/camera/d435/color/image_raw` | `sensor_msgs/msg/Image` | Optional RealSense color image; disabled in current LIO-only configs |

Bridge node:

| Topic | Type | Description |
|---|---|---|
| `raw/odom` | `nav_msgs/msg/Odometry` | Remapped raw FAST-LIVO2 odometry |
| `raw/world_cloud` | `sensor_msgs/msg/PointCloud2` | Remapped raw FAST-LIVO2 world cloud |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry, used in mapping mode |
| `/pgo/offset` | `nav_msgs/msg/Odometry` | PGO loop-closure correction, used in mapping mode |

## Outputs

With default namespace `fastlio2`:

| Topic | Type | Description |
|---|---|---|
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | Processed LIO odometry in `odom` or `map` frame |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | Current scan in `base_link`, with self-filtering |
| `/fastlio2/world_cloud` | `sensor_msgs/msg/PointCloud2` | Current scan in world/map frame |
| `/fastlio2/lio_path` | `nav_msgs/msg/Path` | Processed trajectory |

Raw core topics are remapped to:

| Topic | Type |
|---|---|
| `/fastlio2/raw/odom` | `nav_msgs/msg/Odometry` |
| `/fastlio2/raw/world_cloud` | `sensor_msgs/msg/PointCloud2` |
| `/fastlio2/raw/path` | `nav_msgs/msg/Path` |

Depending on bridge configuration:

| TF | Mode |
|---|---|
| `odom -> base_link` | navigation mode when `publish_tf: true` |
| `map -> odom` | mapping mode when `publish_map_odom: true` |

## Configuration Files

| File | Purpose |
|---|---|
| `config/livo_navigation.yaml` | LIO-only navigation core config, camera disabled |
| `config/bridge_navigation.yaml` | Bridge config for navigation, outputs `odom -> base_link` if enabled |
| `config/livo_mapping.yaml` | LIO-only mapping core config |
| `config/bridge_mapping.yaml` | Bridge config for mapping, publishes `map -> odom` and consumes `/pgo/offset` |
| `config/mid360_realsense.yaml` | FAST-LIVO2 LiDAR + RealSense config |
| `config/camera_realsense_d435i.yaml` | RealSense camera intrinsics |

## Prerequisites

Start the sensors first:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

For visual mode, also start the D435 camera.

Verify:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## Start OpenFlex LIO-Compatible Pipeline

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch fast_livo swerve_lio.launch.py
```

Useful launch arguments:

```bash
ros2 launch fast_livo swerve_lio.launch.py \
  namespace:=fastlio2 \
  core_params_file:=~/openflex_all/openflex_ws/src/openflex_chassis/mapping_localization_layer/fast_livo/config/livo_navigation.yaml \
  bridge_params_file:=~/openflex_all/openflex_ws/src/openflex_chassis/mapping_localization_layer/fast_livo/config/bridge_navigation.yaml
```

For mapping, use `livo_mapping.yaml` and `bridge_mapping.yaml`; `pgo/launch/pgo_launch.py` already does this composition.

## Start Visual Mapping Mode

```bash
ros2 launch fast_livo mapping_mid360_realsense2.launch.py
```

This launch starts `fastlivo_mapping`, image republish, a parameter blackboard, and optionally RViz.

## Check Output

```bash
ros2 topic list | grep fastlio2
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
ros2 topic hz /fastlio2/world_cloud
ros2 run tf2_ros tf2_echo odom base_link
```

## Notes

- The OpenFlex navigation/mapping stack expects `/fastlio2/lio_odom`, `/fastlio2/body_cloud`, and `/fastlio2/world_cloud`.
- `bridge_mapping.yaml` self-filters chassis, lift column, and upper body returns from `body_cloud`.
- The upstream `fastlivo_mapping` node is intentionally kept out of the namespace in `swerve_lio.launch.py`; the bridge provides namespaced OpenFlex topics.
