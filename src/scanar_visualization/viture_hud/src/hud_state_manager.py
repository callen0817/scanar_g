#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from scanar_interfaces.msg import ScanConfidence, CalibrationStatus, TrajectoryStatistics, SystemHealth

class HudStateManager(Node):
    def __init__(self):
        super().__init__('hud_state_manager')

        self.confidence = 100.0
        self.calib_version = "unknown"
        self.calib_ok = False
        
        self.pose_count = 0
        self.distance = 0.0
        self.duration = 0.0
        
        self.cpu_load = 0.0
        self.mem_usage = 0.0

        # Subscriptions
        self.create_subscription(ScanConfidence, '/scanar/scan_confidence_breakdown', self.cb_confidence, 10)
        self.create_subscription(CalibrationStatus, '/scanar/calibration/state', self.cb_calibration, 10)
        self.create_subscription(TrajectoryStatistics, '/scanar/trajectory/statistics', self.cb_trajectory, 10)
        self.create_subscription(SystemHealth, '/scanar/diagnostics/system_health', self.cb_health, 10)

        self.get_logger().info("HUD State Manager initialized.")

    def cb_confidence(self, msg):
        self.confidence = msg.overall

    def cb_calibration(self, msg):
        self.calib_version = msg.calibration_version
        self.calib_ok = msg.slam_allowed

    def cb_trajectory(self, msg):
        self.pose_count = msg.total_poses
        self.distance = msg.total_distance_meters
        self.duration = msg.duration_seconds

    def cb_health(self, msg):
        self.cpu_load = msg.cpu_load
        self.mem_usage = msg.memory_usage

def main(args=None):
    rclpy.init(args=args)
    node = HudStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
