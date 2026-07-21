#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class SessionController(Node):
    def __init__(self):
        super().__init__('session_controller')
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()

        # Service Clients to session_manager
        self.cli_start = self.create_client(Trigger, '/scanar/session/start', callback_group=self.callback_group)
        self.cli_stop = self.create_client(Trigger, '/scanar/session/stop', callback_group=self.callback_group)
        self.cli_pause = self.create_client(Trigger, '/scanar/session/pause', callback_group=self.callback_group)
        self.cli_resume = self.create_client(Trigger, '/scanar/session/resume', callback_group=self.callback_group)
        self.cli_checkpoint = self.create_client(Trigger, '/scanar/session/checkpoint', callback_group=self.callback_group)
        self.cli_reset = self.create_client(Trigger, '/scanar/session/reset', callback_group=self.callback_group)

        # Subscriptions
        self.state_sub = self.create_subscription(String, '/scanar/session/state', self.handle_state, 10, callback_group=self.callback_group)
        self.current_state = "UNKNOWN"

        # Operator Experience service interfaces
        self.srv_op_start = self.create_service(Trigger, '/scanar/operator/start', self.op_start, callback_group=self.callback_group)
        self.srv_op_pause = self.create_service(Trigger, '/scanar/operator/pause', self.op_pause, callback_group=self.callback_group)
        self.srv_op_resume = self.create_service(Trigger, '/scanar/operator/resume', self.op_resume, callback_group=self.callback_group)
        self.srv_op_stop = self.create_service(Trigger, '/scanar/operator/stop', self.op_stop, callback_group=self.callback_group)
        self.srv_op_checkpoint = self.create_service(Trigger, '/scanar/operator/checkpoint', self.op_checkpoint, callback_group=self.callback_group)
        self.srv_op_estop = self.create_service(Trigger, '/scanar/operator/emergency_stop', self.op_estop, callback_group=self.callback_group)

        self.get_logger().info("Operator Session Controller initialized with ReentrantCallbackGroup.")

    def handle_state(self, msg):
        self.current_state = msg.data

    def op_start(self, request, response):
        self.get_logger().info("Operator requested scan START.")
        return self.call_trigger_client(self.cli_start, response)

    def op_pause(self, request, response):
        self.get_logger().info("Operator requested scan PAUSE.")
        return self.call_trigger_client(self.cli_pause, response)

    def op_resume(self, request, response):
        self.get_logger().info("Operator requested scan RESUME.")
        return self.call_trigger_client(self.cli_resume, response)

    def op_stop(self, request, response):
        self.get_logger().info("Operator requested scan STOP / FINALIZATION.")
        return self.call_trigger_client(self.cli_stop, response)

    def op_checkpoint(self, request, response):
        self.get_logger().info("Operator requested scan CHECKPOINT.")
        return self.call_trigger_client(self.cli_checkpoint, response)

    def op_estop(self, request, response):
        self.get_logger().warn("EMERGENCY STOP TRIGGERED!")
        # For security compliance, E-stop triggers reset state machinery
        return self.call_trigger_client(self.cli_reset, response)

    def call_trigger_client(self, client, response):
        if not client.service_is_ready():
            response.success = False
            response.message = f"Core State Machine service is unavailable."
            return response
            
        future = client.call_async(Trigger.Request())
        import time
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
            
        if future.done():
            res = future.result()
            response.success = res.success
            response.message = res.message
        else:
            response.success = False
            response.message = "Service call failed or timed out."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SessionController()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
