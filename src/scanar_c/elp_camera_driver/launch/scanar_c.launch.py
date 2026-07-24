import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. ELP 5MP Global Shutter USB Camera Hardware Driver Node
        Node(
            package='elp_camera_driver',
            executable='elp_camera_node.py',
            name='elp_camera_driver_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'width': 1280,
                'height': 720,
                'fps': 60.0,
            }]
        ),
        # 2. LingBot Reconstruction Engine Node
        Node(
            package='lingbot_engine',
            executable='lingbot_engine_node',
            name='lingbot_engine_node',
            output='screen',
            parameters=[{
                'product': 'scanar_c',
                'camera_type': 'global_shutter',
            }]
        ),
        # 3. VITURE HUD Renderer Node (Optical See-Through AR Display + PIP Stack)
        Node(
            package='viture_hud',
            executable='hud_renderer.py',
            name='hud_renderer',
            output='screen',
            parameters=[{
                'product': 'scanar_c',
            }]
        ),
    ])
