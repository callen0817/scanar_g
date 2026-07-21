# Dependency Report

This report outlines the dependencies identified on the Jetson Orin NX platform for ScanAR Dual.

## ROS 2 System Dependencies
The following standard ROS 2 Humble packages are installed and available:
- `rclpy` (Python client library)
- `std_msgs` (Standard messages)
- `nav_msgs` (Navigation messages, including Odometry)
- `sensor_msgs` (Sensor messages, including Imu, PointCloud2)
- `geometry_msgs` (Geometry messages)
- `cv_bridge` (Bridge between ROS image messages and OpenCV)
- `tf2` & `tf2_ros` (Coordinate transform library)
- `action_msgs` & other communication interfaces

## Python 3 Library Dependencies
Python 3.10 has the following key packages pre-installed:
- `opencv-python` (cv2)
- `numpy`
- `std_msgs` & other ROS Python bindings

## Custom ScanAR Dual Workspace Packages
Located in `~/scanar_dual/src/`:
- `scanar_interfaces` — Core message package containing message definitions such as:
  - `CalibrationMetrics.msg`
  - `CalibrationStatus.msg`
  - `CaptureState.msg`
  - `HardwareStatus.msg`
  - `HudStatistics.msg`
  - `MapStatistics.msg`
  - `RegistrationStatus.msg`
  - `ScanConfidence.msg`
  - `SensorClockStatus.msg`
  - `SessionStatus.msg`
  - `SystemHealth.msg`
  - `TrajectoryStatistics.msg`
