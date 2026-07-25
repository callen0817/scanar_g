import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. VITURE Glasses Hardware Driver Node (Frozen Reference MVP)
        Node(
            package='viture_driver',
            executable='viture_driver_node',
            name='viture_driver',
            output='screen',
            parameters=[{
                'product': 'scanar_g',
                'sim_mode': False,
            }]
        ),
        # 2. LingBot Reconstruction Engine Node
        Node(
            package='lingbot_engine',
            executable='lingbot_engine_node',
            name='lingbot_engine_node',
            output='screen',
            parameters=[{
                'product': 'scanar_g',
            }]
        ),
        # 3. VITURE HUD Renderer Node (Optical See-Through AR Display + PIP Stack)
        Node(
            package='viture_hud',
            executable='hud_renderer.py',
            name='hud_renderer',
            output='screen',
            parameters=[{
                'product': 'scanar_g',
            }]
        ),
    ])
