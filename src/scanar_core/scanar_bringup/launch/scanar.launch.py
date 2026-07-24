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

    import sys
    profiles_dir = "/home/scanarstereo/scanAR_G/src/scanar_core"
    if profiles_dir not in sys.path:
        sys.path.append(profiles_dir)
    from scanar_profiles import get_scanar_profile
    profile = get_scanar_profile(product_val)

    # 1. Hardware Sensor Driver Layer derived directly from plugin profile
    if mode in ['hardware', 'field_test', 'production']:
        sim_mode_val = True if mode == 'field_test' else False
        if "viture_driver" in profile.hardware_drivers:
            nodes.append(Node(
                package='viture_driver',
                executable='viture_driver_node',
                name='viture_driver',
                parameters=[{'sim_mode': sim_mode_val, 'product': product_val}]
            ))
        elif "elp_camera_driver" in profile.hardware_drivers:
            nodes.append(Node(
                package='elp_camera_driver',
                executable='elp_camera_node.py',
                name='elp_camera_driver_node',
                parameters=[{'video_device': '/dev/sensors/camera_elp', 'width': 1920, 'height': 1080, 'fps': 30.0}]
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
