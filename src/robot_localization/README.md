robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

For this rover's self-contained local/global EKFs, raw ZED/ICM gyros, wheel/ZED
odometry and GPS conversion, see [DUAL_EKF.md](DUAL_EKF.md).
All required configurations and Python helpers are installed by this package.

```bash
ros2 launch robot_localization dual_ekf.launch.py publish_approximate_mounts:=true
```
