# pgo

[English](./README.md) | 中文

---

`pgo` 为 OpenFlex 激光建图链路提供位姿图优化、回环检测、地图保存和地图转换工具。

## 推荐使用入口

本包是建图链路中的底层组件。建图、定位、导航等面向用户的功能不建议直接使用此包作为主入口，建议使用我们专门为建图、定位和导航流程设计的 `swerve_navigation` 包。

## PGO 是什么意思

PGO 是 **Pose Graph Optimization**，中文通常称为**位姿图优化**。它把关键帧位姿建模为图中的节点，把里程计约束、回环约束等建模为图中的边，然后通过图优化减少累计漂移，提高地图一致性。

## 包作用

主要可执行文件：

| 可执行文件 | 作用 |
|---|---|
| `pgo_node` | 在线 PGO 与回环检测节点 |
| `pcd_map_publisher` | 将保存的 PCD 地图发布为 PointCloud2 |
| `pcd_to_nav2_map` | 将 3D PCD 地图转换为 Nav2 `map.yaml` + `map.pgm` |
| `rebuild_filtered_map` | 根据保存的关键帧 patch 重建过滤后的地图 |
| `pcd_to_nav2_map_gui` | PCD 转 Nav2 地图的 GUI 工具 |

主要启动文件：

| Launch | 作用 |
|---|---|
| `launch/pgo_launch.py` | 启动 `fast_livo` 建图链路、`pgo_node` 和 RViz |
| `launch/view_saved_map.launch.py` | 发布并可视化已有 PCD 地图 |

## 在线 PGO 输入

| 话题 | 类型 | 说明 |
|---|---|---|
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | LIO 桥接输出的车体坐标系点云 |
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | LIO 里程计 |

配置在 `config/pgo.yaml` 中：

```yaml
cloud_topic: /fastlio2/body_cloud
odom_topic: /fastlio2/lio_odom
```

## 在线 PGO 输出

| 输出 | 类型 | 说明 |
|---|---|---|
| `/pgo/global_cloud` | `sensor_msgs/msg/PointCloud2` | 优化后的全局点云 |
| `/pgo/offset` | `nav_msgs/msg/Odometry` | 回环修正量，供建图桥接使用 |
| `/pgo/loop_markers` | `visualization_msgs/msg/MarkerArray` | 回环可视化 marker |
| `/pgo/save_maps` | `interface/srv/SaveMaps` | 保存优化地图和可选 patch |

## 启动在线 PGO 建图

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch pgo pgo_launch.py
```

该 launch 会组合 `fast_livo/launch/swerve_lio.launch.py`，并使用建图配置：

```text
fast_livo/config/livo_mapping.yaml
fast_livo/config/bridge_mapping.yaml
pgo/config/pgo.yaml
```

## 保存地图

```bash
mkdir -p ~/MID_360_nav/maps/3d/my_map
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '~/MID_360_nav/maps/3d/my_map', save_patches: true}"
```

预期输出目录：

```text
my_map/
├── map.pcd
├── poses.txt
├── patches/
└── scan_context/
```

`patches/`、`poses.txt` 和 `scan_context/` 可用于后续 ICP/ScanContext 重定位和 HBA 地图精修。

## 查看已保存地图

```bash
ros2 launch pgo view_saved_map.launch.py \
  pcd_path:=~/MID_360_nav/maps/3d/my_map/map.pcd
```

可选参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `frame_id` | `map` | 发布点云坐标系 |
| `topic_name` | `/pgo/saved_map_cloud` | 输出话题 |
| `publish_hz` | `1.0` | 发布频率 |
| `voxel_size` | `0.0` | 可选下采样尺寸 |
| `use_rviz` | `true` | 是否启动 RViz |

## 转换 PCD 为 Nav2 2D 地图

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

输出文件：

```text
map.yaml
map.pgm
```

## 重建过滤地图

```bash
ros2 run pgo rebuild_filtered_map \
  --map-dir ~/MID_360_nav/maps/3d/my_map \
  --map-z-min 0.05 \
  --map-z-max 3.0
```

## 检查

```bash
ros2 topic hz /pgo/global_cloud
ros2 topic echo --once /pgo/offset
ros2 service list | grep pgo
```

## 注意事项

- `pgo_launch.py` 面向建图流程，不是普通导航启动入口。
- `bridge_mapping.yaml` 会订阅 `/pgo/offset` 并发布 `map -> odom`。
- 如果后续需要 ICP/ScanContext 重定位，保存地图时建议 `save_patches: true`。
