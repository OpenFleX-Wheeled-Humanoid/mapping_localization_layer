# icp_registration

[English](./README.md) | 中文

---

`icp_registration` 用于基于已有地图的点云配准和重定位。它把实时激光/车体点云与保存的 PCD 地图对齐，发布 `map -> odom`，并结合 ScanContext、NDT、ICP、轮式里程计和可选 GPS 约束恢复定位。

## 包作用

主要可执行文件：

```bash
icp_registration_node
```

主要启动文件：

```bash
launch/icp.launch.py
```

## 输入

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` 或配置的 `pointcloud_topic` | `sensor_msgs/msg/PointCloud2` | 用于配准的实时点云 |
| `/odom` | `nav_msgs/msg/Odometry` | 轮式里程计，用于 yaw 跳变检测和恢复门控 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 手动初始位姿 |
| `/gps/map_position` | `geometry_msgs/msg/PointStamped` | 可选 GPS 辅助 ScanContext 搜索先验 |

## 输出

| 输出 | 类型 | 说明 |
|---|---|---|
| `map -> odom` TF | TF | 主要定位结果 |
| `/localization_hold` | `std_msgs/msg/Empty` | 定位需要保持/拒绝更新时发布 |

## 地图输入

`pcd_path` 必须指向已有的地图 PCD。如果要启用 ScanContext 重定位，地图目录通常需要包含 PGO 保存出的文件：

```text
map_dir/
├── map.pcd
├── poses.txt
├── patches/
└── scan_context/
```

## 配置

主要配置文件：

```bash
config/icp.yaml
```

关键参数：

| 参数 | 含义 |
|---|---|
| `pcd_path` | 保存地图 PCD 路径，使用前必须设置 |
| `pointcloud_topic` | 实时 PointCloud2 话题 |
| `map_frame_id` / `odom_frame_id` / `base_frame_id` | TF 坐标系名 |
| `laser_frame_id` | 输入点云坐标系，当前常用 `base_link` |
| `rough_leaf_size` / `refine_leaf_size` | 粗/精 ICP 下采样尺寸 |
| `xy_offset`, `yaw_offset`, `yaw_resolution` | 初始配准搜索网格 |
| `continuous_realign_interval_sec` | 周期性 ICP 纠偏间隔 |
| `constrain_to_2d` | 是否将 `map -> odom` 约束为平面变换 |
| `wheel_odom_topic` | 用于 yaw 检查的轮式里程计 |
| `localization_hold_topic` | 定位保持/安全通知话题 |

## 前置条件

先启动实时点云来源。在 OpenFlex 导航链路中通常是 `fast_livo` 桥接发布的 `/fastlio2/body_cloud`，但只要配置正确，本包可以订阅任意 `sensor_msgs/msg/PointCloud2` 点云。

## 启动

先编辑 `config/icp.yaml`，或通过参数传入正确的 `pcd_path` 和 `pointcloud_topic`，然后运行：

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch icp_registration icp.launch.py
```

直接运行示例：

```bash
ros2 run icp_registration icp_registration_node --ros-args \
  -p pcd_path:=/path/to/map.pcd \
  -p pointcloud_topic:=/fastlio2/body_cloud
```

## 检查

```bash
ros2 topic hz /fastlio2/body_cloud
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo --once /localization_hold
```

RViz 中可用 `2D Pose Estimate` 手动设置初始位姿，该工具会发布 `/initialpose`。

## 注意事项

- `icp.launch.py` 默认加载 `config/icp.yaml`；仓库默认 `pcd_path` 为空，实车使用前必须设置地图路径。
- 如果实时点云 frame 与 `laser_frame_id` 不一致，节点会尝试通过 TF 做变换。
- 导航中应确保只有一个节点负责发布 `map -> odom`。
