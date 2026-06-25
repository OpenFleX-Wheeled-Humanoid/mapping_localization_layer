# pgo

English | [中文](./README-CN.md)

---

`pgo` provides pose graph optimization, loop closure, map saving, and map conversion tools for the OpenFlex LiDAR mapping pipeline.

## Recommended Entry Point

This package is a lower-level component in the mapping stack. For mapping, localization, navigation, and related end-user workflows, do not directly use this package as the primary entry point. Use the dedicated `swerve_navigation` package instead, which is designed to provide the integrated mapping, localization, and navigation workflows.

## What PGO Means

PGO means **Pose Graph Optimization**. It models keyframe poses as graph nodes and odometry or loop-closure constraints as graph edges, then optimizes the graph to reduce accumulated drift and improve map consistency.

## Package Role

Main executables:

| Executable | Role |
|---|---|
| `pgo_node` | Online PGO and loop closure node |
| `pcd_map_publisher` | Publish a saved PCD map as PointCloud2 |
| `pcd_to_nav2_map` | Convert a 3D PCD map to Nav2 `map.yaml` + `map.pgm` |
| `rebuild_filtered_map` | Rebuild filtered map from saved keyframe patches |
| `pcd_to_nav2_map_gui` | GUI wrapper for PCD to Nav2 map conversion |

Main launch files:

| Launch | Role |
|---|---|
| `launch/pgo_launch.py` | Starts `fast_livo` mapping pipeline, `pgo_node`, and RViz |
| `launch/view_saved_map.launch.py` | Publishes and visualizes an existing PCD map |

## Online PGO Inputs

| Topic | Type | Description |
|---|---|---|
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | Body-frame scan from LIO bridge |
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | LIO odometry |

Configured in `config/pgo.yaml`:

```yaml
cloud_topic: /fastlio2/body_cloud
odom_topic: /fastlio2/lio_odom
```

## Online PGO Outputs

| Output | Type | Description |
|---|---|---|
| `/pgo/global_cloud` | `sensor_msgs/msg/PointCloud2` | Optimized global cloud |
| `/pgo/offset` | `nav_msgs/msg/Odometry` | Loop-closure correction consumed by mapping bridge |
| `/pgo/loop_markers` | `visualization_msgs/msg/MarkerArray` | Loop closure visualization |
| `/pgo/save_maps` | `interface/srv/SaveMaps` | Save optimized map and optional patches |

## Start Online Mapping With PGO

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch pgo pgo_launch.py
```

This launch includes `fast_livo/launch/swerve_lio.launch.py` with mapping configs:

```text
fast_livo/config/livo_mapping.yaml
fast_livo/config/bridge_mapping.yaml
pgo/config/pgo.yaml
```

## Save Map

```bash
mkdir -p ~/MID_360_nav/maps/3d/my_map
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '~/MID_360_nav/maps/3d/my_map', save_patches: true}"
```

Expected output directory:

```text
my_map/
├── map.pcd
├── poses.txt
├── patches/
└── scan_context/
```

`patches/`, `poses.txt`, and `scan_context/` are useful for later ICP/ScanContext relocalization and HBA refinement.

## View Saved Map

```bash
ros2 launch pgo view_saved_map.launch.py \
  pcd_path:=~/MID_360_nav/maps/3d/my_map/map.pcd
```

Optional parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `frame_id` | `map` | Published cloud frame |
| `topic_name` | `/pgo/saved_map_cloud` | Output topic |
| `publish_hz` | `1.0` | Publish frequency |
| `voxel_size` | `0.0` | Optional downsample size |
| `use_rviz` | `true` | Start RViz |

## Convert PCD to Nav2 2D Map

```bash
ros2 run pgo pcd_to_nav2_map \
  --pcd ~/MID_360_nav/maps/3d/my_map/map.pcd \
  --out-dir ~/MID_360_nav/maps/2d/my_map \
  --resolution 0.05 \
  --z-min 0.00 \
  --z-max 2.30 \
  --obstacle-z-min 0.10 \
  --obstacle-z-max 1.40
```

Outputs:

```text
map.yaml
map.pgm
```

## Rebuild Filtered Map

```bash
ros2 run pgo rebuild_filtered_map \
  --map-dir ~/MID_360_nav/maps/3d/my_map \
  --map-z-min 0.05 \
  --map-z-max 3.0
```

## Check

```bash
ros2 topic hz /pgo/global_cloud
ros2 topic echo --once /pgo/offset
ros2 service list | grep pgo
```

## Notes

- `pgo_launch.py` is intended for mapping, not normal navigation.
- `bridge_mapping.yaml` consumes `/pgo/offset` and publishes `map -> odom`.
- Save maps with `save_patches: true` if later ICP/ScanContext relocalization is needed.
