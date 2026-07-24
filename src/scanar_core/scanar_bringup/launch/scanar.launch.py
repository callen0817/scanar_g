import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Configure FastDDS Shared Memory (SHM) zero-copy transport for local nodes
shm_profile = "/home/scanarstereo/scanAR_G/fastdds_shm.xml"
if os.path.exists(shm_profile):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = shm_profile

def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    product_val = LaunchConfiguration('product').perform(context)

    nodes = []

    from viture_driver.product_capabilities import get_product_capability
    capability = get_product_capability(product_val)

    # 1. Hardware Sensor Driver Layer
    if mode in ['hardware', 'field_test', 'production']:
        if capability.has_rgb_camera or "viture_driver" in capability.startup_services:
            sim_mode_val = True if mode == 'field_test' else False
            nodes.append(Node(
                package='viture_driver',
                executable='viture_driver_node',
                name='viture_driver',
                parameters=[{'sim_mode': sim_mode_val, 'product': product_val}]
            ))

    # 2. Capability-Driven Reconstruction Backend Layer
    if mode in ['field_test', 'production']:
        if capability.reconstruction_engine == 'lingbot' or 'lingbot_engine' in capability.startup_services:
            nodes.append(Node(
                package='lingbot_engine',
                executable='lingbot_engine_node',
                name='lingbot_engine',
                parameters=[{'product': product_val}, {'sim_mode': sim_mode_val}]
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
