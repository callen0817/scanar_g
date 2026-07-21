cmake_minimum_required(VERSION 3.8)
project(scanar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanConfidence.msg"
  "msg/CalibrationStatus.msg"
  "msg/HardwareStatus.msg"
  "msg/CaptureState.msg"
  "msg/SessionStatus.msg"
  "msg/TrajectoryStatistics.msg"
  "msg/CalibrationMetrics.msg"
  "msg/HudStatistics.msg"
  "msg/MapStatistics.msg"
  "msg/SystemHealth.msg"
  "msg/SensorClockStatus.msg"
  "msg/RegistrationStatus.msg"
  "msg/GaussianSplat.msg"
  "msg/GaussianSplatArray.msg"
  "msg/ReconstructionFrame.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
