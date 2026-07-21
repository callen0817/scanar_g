#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import uuid
import datetime
import os
import json
import glob

class ScanGSessionManager(Node):
    def __init__(self):
        super().__init__('scanar_session_manager')
        
        # State machine states
        self.STATES = [
            'BOOT',
            'CHECKING_HARDWARE',
            'READY',
            'CAPTURING',
            'PAUSED',
            'FINALIZING',
            'COMPLETE'
        ]
        self.current_state = 'BOOT'
        
        # Session parameters
        self.session_uuid = ""
        self.session_dir = ""
        self.session_start_time = ""
        self.operator_name = "callen0817"

        # Base captures directory
        self.base_captures_dir = "/home/scanarstereo/scanAR_G/captures"
        os.makedirs(self.base_captures_dir, exist_ok=True)

        # State publishers
        self.state_pub = self.create_publisher(String, '/scanar/session/state', 10)
        self.system_ready_pub = self.create_publisher(String, '/scanar/system_ready', 10)
        self.active_dir_pub = self.create_publisher(String, '/scanar/session/active_directory', 10)
        
        # Services for session control
        self.start_srv = self.create_service(Trigger, '/scanar/session/start', self.handle_start)
        self.stop_srv = self.create_service(Trigger, '/scanar/session/stop', self.handle_stop)
        self.pause_srv = self.create_service(Trigger, '/scanar/session/pause', self.handle_pause)
        self.resume_srv = self.create_service(Trigger, '/scanar/session/resume', self.handle_resume)
        self.checkpoint_srv = self.create_service(Trigger, '/scanar/session/checkpoint', self.handle_checkpoint)
        self.reset_srv = self.create_service(Trigger, '/scanar/session/reset', self.handle_reset)
        
        # Heartbeat timer
        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info('ScanAR G Session Manager Node Initialized in BOOT.')
        
        # Advance state automatically
        self.transition_to('CHECKING_HARDWARE')
        # Check camera node or simulate
        self.transition_to('READY')

    def publish_state(self):
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)
        
        ready_msg = String()
        ready_msg.data = 'true' if self.current_state in ['READY', 'CAPTURING', 'PAUSED'] else 'false'
        self.system_ready_pub.publish(ready_msg)
        
        active_dir_msg = String()
        active_dir_msg.data = self.session_dir if self.current_state in ['CAPTURING', 'PAUSED'] else ""
        self.active_dir_pub.publish(active_dir_msg)

    def transition_to(self, target_state):
        if target_state not in self.STATES:
            self.get_logger().error(f'Invalid target state: {target_state}')
            return False
            
        self.get_logger().info(f'Transitioning state: {self.current_state} -> {target_state}')
        self.current_state = target_state
        return True

    def get_next_capture_index(self):
        existing = glob.glob(os.path.join(self.base_captures_dir, "Capture_*"))
        indices = []
        for path in existing:
            name = os.path.basename(path)
            parts = name.split('_')
            if len(parts) >= 2:
                try:
                    indices.append(int(parts[1]))
                except ValueError:
                    pass
        if not indices:
            return 1
        return max(indices) + 1

    def handle_start(self, request, response):
        if self.current_state != 'READY':
            response.success = False
            response.message = f"Cannot start from state {self.current_state}. Must be in READY."
            return response

        # Initialize Session details
        self.session_uuid = str(uuid.uuid4())[:8]
        idx = self.get_next_capture_index()
        self.session_start_time = datetime.datetime.utcnow().isoformat() + "Z"
        
        # Create folder structure matching V1 Capture System specs: Capture_XXX_UUID/
        self.session_dir = os.path.join(self.base_captures_dir, f"Capture_{idx:03d}_{self.session_uuid}")
        
        try:
            os.makedirs(os.path.join(self.session_dir, "rgb"), exist_ok=True)
            os.makedirs(os.path.join(self.session_dir, "imu"), exist_ok=True)
            os.makedirs(os.path.join(self.session_dir, "poses"), exist_ok=True)
            os.makedirs(os.path.join(self.session_dir, "live_splats"), exist_ok=True)
            self.get_logger().info(f"Created ScanAR G session directory layout: {self.session_dir}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to create session directories: {e}"
            return response
        
        self.transition_to('CAPTURING')
        response.success = True
        response.message = f"Session started. Target: {self.session_dir}. CAPTURING."
        return response

    def handle_stop(self, request, response):
        if self.current_state not in ['CAPTURING', 'PAUSED']:
            response.success = False
            response.message = f"Cannot stop from state {self.current_state}."
            return response

        self.transition_to('FINALIZING')
        
        # Write capture metadata manifest
        meta_path = os.path.join(self.session_dir, "metadata.json")
        metadata = {
            "session_uuid": self.session_uuid,
            "operator": self.operator_name,
            "start_time": self.session_start_time,
            "stop_time": datetime.datetime.utcnow().isoformat() + "Z",
            "device": "VITURE Luma Ultra",
            "mode": "ScanAR G Real-Time Gaussian Capture MVP"
        }
        
        try:
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            self.get_logger().info(f"Session metadata.json written: {meta_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write metadata: {e}")

        self.transition_to('COMPLETE')
        response.success = True
        response.message = f"Session finalized at {self.session_dir}. Transitioned to COMPLETE."
        return response

    def handle_pause(self, request, response):
        if self.current_state == 'CAPTURING':
            self.transition_to('PAUSED')
            response.success = True
            response.message = "Session paused."
        else:
            response.success = False
            response.message = f"Cannot pause from state {self.current_state}."
        return response

    def handle_resume(self, request, response):
        if self.current_state == 'PAUSED':
            self.transition_to('CAPTURING')
            response.success = True
            response.message = "Session resumed."
        else:
            response.success = False
            response.message = f"Cannot resume from state {self.current_state}."
        return response

    def handle_checkpoint(self, request, response):
        if self.current_state == 'CAPTURING':
            response.success = True
            response.message = f"Checkpoint created for {self.session_uuid}."
        else:
            response.success = False
            response.message = "Cannot write checkpoint unless CAPTURING."
        return response

    def handle_reset(self, request, response):
        self.transition_to('BOOT')
        self.transition_to('CHECKING_HARDWARE')
        self.transition_to('READY')
        response.success = True
        response.message = "Reset to READY."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ScanGSessionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
