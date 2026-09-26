# fast_livo

[English](./README.md) | 中文

---

`fast_livo` 包含基于 FAST-LIVO2 的激光/视觉惯性建图节点，以及 OpenFlex 兼容桥接节点。当前底盘系统中，它主要作为 FAST-LIO 兼容的数据链路，用于建图和导航。

## 包作用

主要可执行文件：

| 可执行文件 | 作用 |
|---|---|
| `fastlivo_mapping` | FAST-LIVO2 核心节点，订阅 MID360 LiDAR/IMU 和可选相机 |
| `lio_bridge_node` | 将 FAST-LIVO2 原始输出转换为 OpenFlex 使用的 `/fastlio2/*` 话题和 TF 行为 |

主要启动文件：

| Launch | 作用 |
|---|---|
| `launch/swerve_lio.launch.py` | OpenFlex LIO 兼容链路，包含核心节点和桥接节点 |
| `launch/mapping_mid360_realsense2.launch.py` | MID360 + RealSense 视觉激光建图启动 |

## 输入

FAST-LIVO2 核心节点：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `livox_interfaces2/msg/CustomMsg` | MID360 点云 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 IMU |
| `/camera/d435/color/image_raw` | `sensor_msgs/msg/Image` | 可选 RealSense 彩色图像；当前 LIO-only 配置中关闭 |

桥接节点：

| 话题 | 类型 | 说明 |
|---|---|---|
| `raw/odom` | `nav_msgs/msg/Odometry` | 重映射后的 FAST-LIVO2 原始里程计 |
| `raw/world_cloud` | `sensor_msgs/msg/PointCloud2` | 重映射后的 FAST-LIVO2 原始世界点云 |
| `/odom` | `nav_msgs/msg/Odometry` | 轮式里程计，建图模式使用 |
| `/pgo/offset` | `nav_msgs/msg/Odometry` | PGO 回环修正，建图模式使用 |

## 输出

默认 namespace 为 `fastlio2`：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | 处理后的 LIO 里程计 |
| `/fastlio2/body_cloud` | `sensor_msgs/msg/PointCloud2` | 变换到 `base_link` 的当前帧点云，带自车点过滤 |
| `/fastlio2/world_cloud` | `sensor_msgs/msg/PointCloud2` | 世界/地图坐标系下当前帧点云 |
| `/fastlio2/lio_path` | `nav_msgs/msg/Path` | 处理后的轨迹 |

核心节点原始输出会被重映射为：

| 话题 | 类型 |
|---|---|
| `/fastlio2/raw/odom` | `nav_msgs/msg/Odometry` |
| `/fastlio2/raw/world_cloud` | `sensor_msgs/msg/PointCloud2` |
| `/fastlio2/raw/path` | `nav_msgs/msg/Path` |

根据桥接配置不同，可能发布：

| TF | 模式 |
|---|---|
| `odom -> base_link` | 导航模式，`publish_tf: true` |
| `map -> odom` | 建图模式，`publish_map_odom: true` |

## 配置文件

| 文件 | 用途 |
|---|---|
| `config/livo_navigation.yaml` | LIO-only 导航核心配置，相机关闭 |
| `config/bridge_navigation.yaml` | 导航桥接配置，按需输出 `odom -> base_link` |
| `config/livo_mapping.yaml` | LIO-only 建图核心配置 |
| `config/bridge_mapping.yaml` | 建图桥接配置，发布 `map -> odom`，订阅 `/pgo/offset` |
| `config/mid360_realsense.yaml` | FAST-LIVO2 LiDAR + RealSense 配置 |
| `config/camera_realsense_d435i.yaml` | RealSense 相机内参 |

## 前置条件

先启动传感器：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

如果使用视觉模式，还需要启动 D435 相机。

检查输入：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 启动 OpenFlex LIO 兼容链路

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch fast_livo swerve_lio.launch.py
```

常用启动参数：

```bash
ros2 launch fast_livo swerve_lio.launch.py \
  namespace:=fastlio2 \
  core_params_file:=~/openflex_all/openflex_ws/src/openflex_chassis/mapping_localization_layer/fast_livo/config/livo_navigation.yaml \
  bridge_params_file:=~/openflex_all/openflex_ws/src/openflex_chassis/mapping_localization_layer/fast_livo/config/bridge_navigation.yaml
```

建图时使用 `livo_mapping.yaml` 和 `bridge_mapping.yaml`；`pgo/launch/pgo_launch.py` 已经按这个方式组合。

## 启动视觉建图模式

```bash
ros2 launch fast_livo mapping_mid360_realsense2.launch.py
```

该 launch 会启动 `fastlivo_mapping`、图像 republish、参数 blackboard，以及可选 RViz。

## 检查输出

```bash
ros2 topic list | grep fastlio2
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
ros2 topic hz /fastlio2/world_cloud
ros2 run tf2_ros tf2_echo odom base_link
```

## 注意事项

- OpenFlex 导航/建图链路主要依赖 `/fastlio2/lio_odom`、`/fastlio2/body_cloud` 和 `/fastlio2/world_cloud`。
- `bridge_mapping.yaml` 会过滤底盘、升降柱和上半身结构在 `body_cloud` 中造成的自车点。
- `swerve_lio.launch.py` 中故意让上游 `fastlivo_mapping` 不进 namespace，避免上游节点命名空间导致崩溃；由桥接节点提供 namespaced 的 OpenFlex 话题。
