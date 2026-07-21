#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import time
import sys

def test_services():
    print("Starting session_manager_node...")
    cmd = ["bash", "-c", "source /opt/ros/humble/setup.bash && source /home/scanarstereo/scanAR_G/install/setup.bash && ros2 run session_manager session_manager_node"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(2)
    
    rclpy.init()
    node = Node('test_session_services')
    
    cli_start = node.create_client(Trigger, '/scanar/session/start')
    cli_stop = node.create_client(Trigger, '/scanar/session/stop')
    
    print("Waiting for start service...")
    if not cli_start.wait_for_service(timeout_sec=5.0):
        print("FAIL: start service not available!")
        proc.terminate()
        return
        
    print("Calling start service...")
    future = cli_start.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    print(f"Start Response: success={res.success}, message='{res.message}'")
    
    time.sleep(2)
    
    print("Calling stop service...")
    future = cli_stop.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    print(f"Stop Response: success={res.success}, message='{res.message}'")
    
    proc.terminate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    test_services()
