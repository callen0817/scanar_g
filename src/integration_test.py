#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import time
import subprocess
import os
import shutil
import sys

def run_integration_test():
    print("=== STARTING SCANAR G SYSTEM INTEGRATION TEST ===")
    
    # 1. Start the ROS2 launch file in the background
    print("Launching ScanAR G system in field_test mode...")
    log_file = open("/home/scanarstereo/scanAR_G/integration_test_stack.log", "w")
    cmd = ["bash", "-c", "source /opt/ros/humble/setup.bash && source /home/scanarstereo/scanAR_G/install/setup.bash && ros2 launch scanar_bringup scanar.launch.py mode:=field_test"]
    proc = subprocess.Popen(cmd, stdout=log_file, stderr=log_file, preexec_fn=os.setsid)
    
    # Wait for nodes to initialize
    time.sleep(5)
    
    # Initialize rclpy to publish start/stop commands
    rclpy.init()
    node = Node('integration_test_node')
    key_pub = node.create_publisher(String, '/scanar/operator/keyboard_input', 10)
    
    time.sleep(2)
    
    # 2. Trigger START scan (equivalent to pressing SPACE)
    print("Sending keyboard event: SPACE to start capture...")
    msg = String()
    msg.data = "SPACE"
    key_pub.publish(msg)
    
    # Let it capture for 6 seconds
    print("Capturing data (simulated walking, feature extraction, splat accumulation)...")
    for i in range(6):
        time.sleep(1)
        print(f"  Sec {i+1}...")
        
    # 3. Trigger STOP & Finalize (equivalent to pressing S)
    print("Sending keyboard event: S to stop capture...")
    msg.data = "S"
    key_pub.publish(msg)
    
    # Wait for finalization and export
    time.sleep(3)
    
    # Cleanup ROS2 node
    node.destroy_node()
    rclpy.shutdown()
    
    # Terminate the ROS2 launch process group
    print("Stopping ROS2 stack...")
    import signal
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    proc.wait()
    
    # 4. Verify deliverables
    print("\n=== VERIFYING DELIVERABLES ===")
    captures_dir = "/home/scanarstereo/scanAR_G/captures"
    if not os.path.exists(captures_dir):
        print("FAIL: captures directory does not exist!")
        return False
        
    sessions = os.listdir(captures_dir)
    print(f"Found sessions in captures: {sessions}")
    if not sessions:
        print("FAIL: No session directory created!")
        return False
        
    session_dir = os.path.join(captures_dir, sessions[0])
    
    # Check folders
    required_dirs = ["rgb", "imu", "poses", "live_splats"]
    for rd in required_dirs:
        p = os.path.join(session_dir, rd)
        if not os.path.exists(p):
            print(f"FAIL: Required directory missing: {p}")
            return False
            
    # Check files
    required_files = [
        "metadata.json",
        "scene.splat",
        "scene.ply",
        os.path.join("poses", "poses.csv"),
        os.path.join("poses", "poses.tum"),
        os.path.join("imu", "imu_raw.csv")
    ]
    for rf in required_files:
        p = os.path.join(session_dir, rf)
        if not os.path.exists(p):
            print(f"FAIL: Required file missing: {p}")
            return False
        print(f"✓ Found: {rf} (size: {os.path.getsize(p)} bytes)")
        
    print("\nSUCCESS: All files and folder structures matching the V1 roadmap are present and verified!")
    return True

if __name__ == "__main__":
    success = run_integration_test()
    if success:
        sys.exit(0)
    else:
        sys.exit(1)
