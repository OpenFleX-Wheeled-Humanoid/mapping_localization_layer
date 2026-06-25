# Mapping and Localization Layer

English | [中文](./README.zh-CN.md)

---

![Cover](./image/cover.gif)


This layer provides LiDAR/IMU mapping, LiDAR-inertial-visual mapping, loop-closure optimization, map refinement, point-cloud relocalization, and `map -> odom` correction.

## Packages

- `fastlio2`: MID-360 LiDAR-inertial odometry for high-rate odometry, point-cloud mapping, and mapping-mode `map -> odom`.
- `fast_livo（not used）`: LiDAR-inertial-visual mapping with MID-360 and D435.
- `pgo`: Pose graph optimization using keyframes, ScanContext, NDT/ICP, and GTSAM iSAM2.
- `hba`: Hierarchical bundle adjustment for offline or service-triggered map refinement.
- `icp_registration`: PCD-map relocalization with NDT/ICP/ScanContext and smoothed `map -> odom` correction.

## Typical Flow

`livox_ros_driver2` provides point cloud and IMU data. `fastlio2` or `fast_livo` estimates local motion and builds point clouds. During mapping, `pgo` optimizes keyframes with loop closures. Saved maps can be refined by `hba`. During navigation, `icp_registration` loads the PCD map and aligns live point clouds to correct localization.

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
