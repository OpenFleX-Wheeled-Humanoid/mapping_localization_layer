# hba

[English](./README.md) | 中文

---

`hba` 用于离线 LiDAR 地图精修，核心算法是 Hierarchical Bundle Adjustment。它读取保存的关键帧 patch 和位姿，进行优化，并可发布精修后的地图点云用于可视化。

## 包作用

主要可执行文件：

```bash
hba_node
```

主要启动文件：

```bash
launch/hba_launch.py
```

## 输入

HBA 通过服务触发。输入应是由 `pgo/save_maps` 保存、且 `save_patches: true` 的地图目录：

```text
map_dir/
├── poses.txt
└── patches/
    ├── 0.pcd
    ├── 1.pcd
    └── ...
```

## 服务

节点启动在 `hba` namespace 下，因此服务名通常为：

| 服务 | 类型 | 说明 |
|---|---|---|
| `/hba/refine_map` | `interface/srv/RefineMap` | 读取 `maps_path` 并执行 HBA 优化 |
| `/hba/save_poses` | `interface/srv/SavePoses` | 保存优化后的位姿 |

服务格式：

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

## 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/hba/map_points` | `sensor_msgs/msg/PointCloud2` | 精修后的地图点云，frame 为 `map` |

## 配置

主要配置文件：

```bash
config/hba.yaml
```

关键参数：

| 参数 | 含义 |
|---|---|
| `scan_resolution` | 发布前的下采样分辨率 |
| `window_size` | 优化窗口大小 |
| `stride` | 窗口步长 |
| `voxel_size` | HBA 使用的体素尺寸 |
| `min_point_num` | 有效地图元素最小点数 |
| `max_layer` | HBA 层级深度 |
| `plane_thresh` | 平面残差阈值 |
| `ba_max_iter` | 内层 BA 最大迭代次数 |
| `hba_iter` | HBA 优化轮数 |
| `down_sample` | 地图下采样尺寸 |

## 启动

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch hba hba_launch.py
```

## 精修保存地图

```bash
ros2 service call /hba/refine_map interface/srv/RefineMap \
  "{maps_path: '~/MID_360_nav/maps/3d/my_map'}"
```

保存优化后的位姿：

```bash
ros2 service call /hba/save_poses interface/srv/SavePoses \
  "{file_path: '~/MID_360_nav/maps/3d/my_map/hba_poses.txt'}"
```

## 检查

```bash
ros2 service list | grep hba
ros2 topic echo --once /hba/map_points
```

如果通过 `hba_launch.py` 启动了 RViz，可以添加 `/hba/map_points` 的 PointCloud2 显示。

## 注意事项

- 保存 PGO 地图时应设置 `save_patches: true`，HBA 需要 `poses.txt` 和 `patches/` 目录。
- HBA 是建图后的离线精修工具，普通导航时不需要启动。
- 节点只有在 `/hba/map_points` 有订阅者时才发布地图点云。
