import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    product_val = LaunchConfiguration('product').perform(context)

    nodes = []

    # 1. VITURE Sensor Driver Layer
    if mode in ['hardware', 'field_test', 'production']:
        sim_mode_val = True if mode == 'field_test' else False
        nodes.append(Node(
            package='viture_driver',
            executable='viture_driver_node',
            name='viture_driver',
            parameters=[{'sim_mode': sim_mode_val}]
        ))

    # 2. Reconstruction Backend Layer
    if mode in ['field_test', 'production']:
        if product_val == 'scanar_g':
            nodes.append(Node(
                package='lingbot_backend',
                executable='lingbot_backend_node',
                name='lingbot_backend',
                parameters=[{'product': product_val}, {'sim_mode': sim_mode_val}]
            ))
        else:
            nodes.append(Node(
                package='vigs_backend',
                executable='vigs_backend_node',
                name='vigs_backend'
            ))

    # 3. Capture and Session Layer
    if mode in ['capture', 'field_test', 'production']:
        nodes.append(Node(
            package='session_manager',
            executable='session_manager_node',
            name='session_manager'
        ))
        nodes.append(Node(
            package='continuous_capture',
            executable='continuous_capture_node',
            name='continuous_capture'
        ))

    # 4. Operator Experience Layer
    if mode in ['field_test', 'production']:
        nodes.append(Node(
            package='scanar_operator',
            executable='session_controller.py',
            name='session_controller'
        ))
        nodes.append(Node(
            package='scanar_operator',
            executable='input_manager.py',
            name='input_manager'
        ))

    # 5. Visualizations HUD & Console Dashboards Layer
    if mode in ['field_test', 'production']:
        nodes.append(Node(
            package='viture_hud',
            executable='hud_renderer.py',
            name='hud_renderer',
            parameters=[{'product': product_val}]
        ))
        nodes.append(Node(
            package='scanar_diagnostics_dashboard',
            executable='dashboard_node.py',
            name='diagnostics_dashboard',
            output='screen',
            parameters=[{'product': product_val}]
        ))

    return nodes

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='production',
        description='Execution mode: hardware, capture, field_test, production'
    )
    product_arg = DeclareLaunchArgument(
        'product',
        default_value='scanar_g',
        description='Capture product: scanar_g, scanar_s, scanar_s2, scanar_l, scanar_l2, scanar_pro'
    )

    return LaunchDescription([
        mode_arg,
        product_arg,
        OpaqueFunction(function=launch_setup)
    ])
