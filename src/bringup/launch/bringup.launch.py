import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context):
    # Launch arguments
    port_arg = LaunchConfiguration('port')
    
    # Get namespace value 
    namespace_value = context.launch_configurations.get('namespace', '')
    
    # Get package paths
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        controller_package_path = get_package_share_directory('controller')
        app_package_path = get_package_share_directory('app')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py'))
    )
    
    # depth_camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    # )

    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
    # )

    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': port_arg}],
        output='screen'
    )

    # web_video_server_node = Node(
    #     package='web_video_server',
    #     executable='web_video_server',
    #     output='screen',
    # )

    # start_app_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(app_package_path, 'launch/start_app.launch.py')),
    #     launch_arguments={
    #         'namespace': namespace,
    #         'use_namespace': str(use_namespace).lower()
    #     }.items(),
    # )

    init_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_package_path, 'launch/init_pose.launch.py')),
        launch_arguments={
            'action_name': 'init',
        }.items(),
    )

    # joystick_control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),
    # )

    # startup_check_node = Node(
    #     package='bringup',
    #     executable='startup_check',
    #     name='startup_check',
    #     namespace=namespace,
    #     output='screen',
    # )

    nodes_to_launch = [
        controller_launch,
        rosbridge_websocket_node,
        init_pose_launch,
    ]
    
    # Apply namespace if provided
    if namespace_value:
        return [
            GroupAction(actions=[
                PushRosNamespace(namespace_value),
                *nodes_to_launch
            ])
        ]
    else:
        return nodes_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace (e.g. robot1, robot2). Leave empty for no namespace.'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='9091',
            description='Port for rosbridge websocket server'
        ),
        OpaqueFunction(function = launch_setup)
    ])


