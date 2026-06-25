# icp_registration

English | [中文](./README-CN.md)

---

`icp_registration` performs map-based point cloud registration and relocalization. It aligns live LiDAR/body clouds against a saved PCD map, publishes `map -> odom`, and can use ScanContext, NDT, ICP, wheel odometry, and optional GPS constraints to recover localization.

## Package Role

Main executable:

```bash
icp_registration_node
```

Main launch:

```bash
launch/icp.launch.py
```

## Inputs

| Topic | Type | Description |
|---|---|---|
| `/livox/lidar` or configured `pointcloud_topic` | `sensor_msgs/msg/PointCloud2` | Live point cloud to align |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry for yaw-jump detection and recovery gating |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Manual initial pose from RViz |
| `/gps/map_position` | `geometry_msgs/msg/PointStamped` | Optional GPS-assisted ScanContext search prior |

## Outputs

| Output | Type | Description |
|---|---|---|
| `map -> odom` TF | TF | Main localization result |
| `/localization_hold` | `std_msgs/msg/Empty` | Published when localization should be held/rejected by safety logic |

## Map Inputs

`pcd_path` must point to an existing map PCD. For ScanContext relocalization, the map directory should also contain the files produced by PGO map saving:

```text
map_dir/
├── map.pcd
├── poses.txt
├── patches/
└── scan_context/
```

## Configuration

Main config file:

```bash
config/icp.yaml
```

Important parameters:

| Parameter | Meaning |
|---|---|
| `pcd_path` | Path to saved map PCD; must be set before use |
| `pointcloud_topic` | Live PointCloud2 topic |
| `map_frame_id` / `odom_frame_id` / `base_frame_id` | TF frame names |
| `laser_frame_id` | Frame of the incoming cloud, currently commonly `base_link` |
| `rough_leaf_size` / `refine_leaf_size` | Downsample sizes for rough/refined ICP |
| `xy_offset`, `yaw_offset`, `yaw_resolution` | Search grid for initial alignment |
| `continuous_realign_interval_sec` | Periodic ICP correction interval |
| `constrain_to_2d` | Keep `map -> odom` planar |
| `wheel_odom_topic` | Wheel odometry topic used for yaw checks |
| `localization_hold_topic` | Hold/safety notification topic |

## Prerequisites

Start a live point cloud source first. In the OpenFlex navigation stack this is usually `/fastlio2/body_cloud` from `fast_livo` bridge, but this package can consume any `sensor_msgs/msg/PointCloud2` cloud if configured.

## Start

Edit `config/icp.yaml` or pass parameters so `pcd_path` and `pointcloud_topic` are correct, then run:

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch icp_registration icp.launch.py
```

Direct run example:

```bash
ros2 run icp_registration icp_registration_node --ros-args \
  -p pcd_path:=/path/to/map.pcd \
  -p pointcloud_topic:=/fastlio2/body_cloud
```

## Check

```bash
ros2 topic hz /fastlio2/body_cloud
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo --once /localization_hold
```

Manual initial pose can be set in RViz using `2D Pose Estimate`, which publishes `/initialpose`.

## Notes

- `icp.launch.py` uses `config/icp.yaml`; the checked-in default has an empty `pcd_path`, so a map path must be provided before real use.
- If the live cloud frame differs from `laser_frame_id`, the node tries to use TF to transform it.
- For navigation, keep only one node responsible for `map -> odom`.
