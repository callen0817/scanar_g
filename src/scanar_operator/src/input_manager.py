#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class InputManager(Node):
    def __init__(self):
        super().__init__('input_manager')

        # Service Clients to session_controller
        self.cli_op_start = self.create_client(Trigger, '/scanar/operator/start')
        self.cli_op_pause = self.create_client(Trigger, '/scanar/operator/pause')
        self.cli_op_resume = self.create_client(Trigger, '/scanar/operator/resume')
        self.cli_op_stop = self.create_client(Trigger, '/scanar/operator/stop')
        self.cli_op_checkpoint = self.create_client(Trigger, '/scanar/operator/checkpoint')
        self.cli_op_estop = self.create_client(Trigger, '/scanar/operator/emergency_stop')

        # Subscriptions
        self.key_sub = self.create_subscription(String, '/scanar/operator/keyboard_input', self.handle_keyboard, 10)
        self.get_logger().info("Operator Input Manager initialized. Listening to key events on /scanar/operator/keyboard_input.")

    def handle_keyboard(self, msg):
        key = msg.data.upper().strip()
        self.get_logger().info(f"Keyboard event captured: {key}")

        if key == "SPACE":
            # Start or Stop depending on the current session state
            # For validation, we trigger START
            self.call_service(self.cli_op_start)
        elif key == "P":
            self.call_service(self.cli_op_pause)
        elif key == "R":
            self.call_service(self.cli_op_resume)
        elif key == "S":
            self.call_service(self.cli_op_stop)
        elif key == "C":
            self.call_service(self.cli_op_checkpoint)
        elif key == "ESC":
            self.call_service(self.cli_op_estop)

    def call_service(self, client):
        if not client.service_is_ready():
            self.get_logger().warn("Session Controller service not ready.")
            return
        client.call_async(Trigger.Request())

def main(args=None):
    rclpy.init(args=args)
    node = InputManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
