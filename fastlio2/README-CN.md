# fastlio2

[English](./README.md) | 中文

---

`fastlio2` 是 OpenFlex 底盘使用的轻量 FAST-LIO2 激光惯性里程计包。它订阅 Livox MID360 点云和 IMU，估计雷达/车体里程计，并发布里程计、当前帧点云、轨迹以及可选 TF。

## 包作用

当只需要直接运行 FAST-LIO2 里程计、不需要 FAST-LIVO 兼容桥接时，使用这个包。

主要可执行文件：

```bash
lio_node
```

主要启动文件：

```bash
launch/lio_launch.py
```

## 输入

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `livox_interfaces2/msg/CustomMsg` | MID360 点云 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 内置 IMU |
| `/odom` | `nav_msgs/msg/Odometry` | 轮式里程计，建图模式用于静止检测和 map->odom 计算 |
| `/pgo/offset` | `nav_msgs/msg/Odometry` | PGO 回环修正，只在 map->odom 模式下使用 |

## 输出

launch 中节点 namespace 为 `fastlio2`，因此相对话题会出现在 `/fastlio2` 下。

| 话题 | 类型 | 说明 |
|---|---|---|
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | LIO 里程计 |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | 当前帧点云，变换到车体坐标系 |
| `/fastlio2/world_cloud` | `sensor_msgs/msg/PointCloud2` | 当前帧点云，变换到世界/地图坐标系 |
| `/fastlio2/lio_path` | `nav_msgs/msg/Path` | 估计轨迹 |

根据配置不同，节点也可能发布 TF：

| TF | 使用模式 |
|---|---|
| `odom -> base_link` | `publish_tf` 为 true 的导航/里程计模式 |
| `map -> odom` | `publish_map_odom` 为 true 的建图模式 |

## 配置文件

| 文件 | 用途 |
|---|---|
| `config/lio.yaml` | 默认独立 LIO 配置 |
| `config/lio_navigation.yaml` | 导航配置，`world_frame=odom`，`publish_tf=false` |
| `config/lio_navigation_gps.yaml` | GPS/EKF 接管 TF 的导航配置 |
| `config/lio_mapping.yaml` | 建图配置，`world_frame=map`，支持发布 `map -> odom` 和 PGO 修正 |

关键参数：

| 参数 | 含义 |
|---|---|
| `imu_topic` / `lidar_topic` | MID360 输入话题 |
| `body_frame` | 输出车体坐标系，通常为 `base_link` 或 `mid360_link` |
| `world_frame` | 导航用 `odom`，建图用 `map` |
| `publish_tf` | 是否发布车体 TF |
| `publish_map_odom` | 是否发布 `map -> odom` |
| `r_ib` / `t_ib` | IMU body 到 `base_link` 的外参 |
| `r_il` / `t_il` | LiDAR 到 IMU 的外参 |

## 前置条件

先启动 MID360 驱动：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

确认数据正常：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 启动

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch fastlio2 lio_launch.py
```

`lio_launch.py` 默认加载 `config/lio.yaml`，并使用 `rviz/fastlio2.rviz` 启动 RViz。

## 直接运行节点

```bash
ros2 run fastlio2 lio_node --ros-args \
  -p config_path:=~/openflex_all/openflex_ws/src/openflex_chassis/mapping_localization_layer/fastlio2/config/lio_navigation.yaml
```

## 检查输出

```bash
ros2 topic list | grep fastlio2
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
ros2 topic echo --once /fastlio2/lio_odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 注意事项

- `livox_ros_driver2` 发布的 MID360 IMU 加速度实际单位是 `g`，本节点通过内部 `imu_accel_scale` 做尺度处理。
- 如果导航系统中已有其他节点负责 `odom -> base_link`，应保持 `publish_tf: false`，避免 TF 冲突。
- 如果需要回环检测和地图保存，建议使用上层建图 launch 或 `fast_livo` + `pgo` 组合。
