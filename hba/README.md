# hba

English | [中文](./README-CN.md)

---

`hba` performs offline Hierarchical Bundle Adjustment for LiDAR map refinement. It loads saved keyframe patches and poses, optimizes them, and can publish refined map points for visualization.

## Package Role

Main executable:

```bash
hba_node
```

Main launch:

```bash
launch/hba_launch.py
```

## Inputs

HBA is service-driven. It expects a map directory produced by `pgo/save_maps` with patches enabled:

```text
map_dir/
├── poses.txt
└── patches/
    ├── 0.pcd
    ├── 1.pcd
    └── ...
```

## Services

The node is launched under namespace `hba`, so service names are usually:

| Service | Type | Description |
|---|---|---|
| `/hba/refine_map` | `interface/srv/RefineMap` | Load `maps_path`, run HBA optimization |
| `/hba/save_poses` | `interface/srv/SavePoses` | Save optimized poses to a file |

Service request formats:

```text
RefineMap.srv:
string maps_path
---
bool success
string message
```

```text
SavePoses.srv:
string file_path
---
bool success
string message
```

## Outputs

| Topic | Type | Description |
|---|---|---|
| `/hba/map_points` | `sensor_msgs/msg/PointCloud2` | Refined map points for visualization, frame `map` |

## Configuration

Main config:

```bash
config/hba.yaml
```

Important parameters:

| Parameter | Meaning |
|---|---|
| `scan_resolution` | Downsample resolution before publishing |
| `window_size` | Optimization window size |
| `stride` | Window stride |
| `voxel_size` | Voxel size used by HBA |
| `min_point_num` | Minimum points for valid map elements |
| `max_layer` | HBA hierarchy depth |
| `plane_thresh` | Plane residual threshold |
| `ba_max_iter` | Inner bundle adjustment iterations |
| `hba_iter` | Number of HBA optimization passes |
| `down_sample` | Map downsample size |

## Start

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch hba hba_launch.py
```

## Refine a Saved Map

```bash
ros2 service call /hba/refine_map interface/srv/RefineMap \
  "{maps_path: '~/MID_360_nav/maps/3d/my_map'}"
```

Save optimized poses:

```bash
ros2 service call /hba/save_poses interface/srv/SavePoses \
  "{file_path: '~/MID_360_nav/maps/3d/my_map/hba_poses.txt'}"
```

## Check

```bash
ros2 service list | grep hba
ros2 topic echo --once /hba/map_points
```

If RViz is running from `hba_launch.py`, add `/hba/map_points` as a PointCloud2 display.

## Notes

- Run PGO map saving with `save_patches: true`; HBA needs both `poses.txt` and the `patches/` directory.
- HBA is a post-processing/refinement tool. It is not required during normal navigation.
- The node only publishes map points when there is at least one subscriber.
