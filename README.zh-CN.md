# 建图定位层

[English](./README.md) | 中文

---

![封面](./image/cover.gif)


本层负责 LiDAR/IMU 建图、视觉-惯性-雷达建图、回环优化、地图精修、点云重定位和 `map -> odom` 校正。

## 包清单

- `fastlio2`: MID-360 LiDAR-inertial odometry，提供高频里程计、局部/全局点云和建图模式下的 `map -> odom`。
- `fast_livo(暂时未使用)`: LiDAR-inertial-visual mapping，结合 MID-360 与 D435，用于更丰富的建图链路。
- `pgo`: Pose Graph Optimization，基于关键帧、ScanContext、NDT/ICP 和 GTSAM iSAM2 做回环优化。
- `hba`: Hierarchical Bundle Adjustment，用于离线或服务触发的点云地图精修。
- `icp_registration`: 基于 PCD 地图的 NDT/ICP/ScanContext 重定位，发布平滑的 `map -> odom` 校正。

## 典型数据流

`livox_ros_driver2` 发布点云/IMU，`fastlio2` 或 `fast_livo` 估计局部运动和点云地图。建图时 `pgo` 接收 LIO 输出并优化关键帧；保存地图后 `hba` 可进一步精修。导航时 `icp_registration` 加载 PCD 地图，对实时点云做重定位并校正 TF。

## 启动入口
- `fastlio2/launch/lio_launch.py`
- `pgo/launch/pgo_launch.py`
- `hba/launch/hba_launch.py`
- `icp_registration/launch/icp.launch.py`

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
