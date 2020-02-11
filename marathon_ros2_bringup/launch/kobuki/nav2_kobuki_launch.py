from launch import LaunchDescription
from launch_ros.actions import Node
import launch.substitutions
import launch_ros.actions
import os
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    hardware_config = Path(get_package_share_directory('pilot_kobuki'), 'config', 'hardware.yaml')
    assert hardware_config.is_file()

    kobuki_dir = get_package_share_directory('pilot_kobuki')
    launch_dir = os.path.join(kobuki_dir, 'launch')

    marathon_dir = get_package_share_directory('marathon_ros2_bringup')
    marathon_launch_dir = os.path.join(marathon_dir, 'launch')

    #enable_align_depth = launch.substitutions.LaunchConfiguration('enable_aligned_depth', default="false")

    # Create the launch configuration variables
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(marathon_dir, 'params', 'nav2_marathon_kobuki_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rplidar_ros'),
            'launch',
            'rplidar.launch.py'))
        )

    #realsense_node = launch_ros.actions.Node(
    #        package='realsense_ros2_camera',
    #        node_executable='realsense_ros2_camera',
    #        node_name='realsense_ros2_camera',
    #        output='screen')


    kobuki_node = launch_ros.actions.Node(
      package='turtlebot2_drivers',
      node_executable='kobuki_node',
      output='screen')

    tf_kobuki2laser_node = launch_ros.actions.Node(
      package='tf2_ros',
      node_executable='static_transform_publisher',
      output='screen',
      arguments=['0.11', '0.0', '0.17',
                 '0', '1', '0', '0',
                 'base_link',
                 'laser_frame'])

    laserfilter_node = launch_ros.actions.Node(
      package='pilot_kobuki',
      node_executable='laser_filter_node')

    tf_kobuki2imu_node = launch_ros.actions.Node(
      package='tf2_ros',
      node_executable='static_transform_publisher',
      output='screen',
      arguments=['0.0', '0.0', '0.0',
                 '0', '0', '0', '1',
                 'base_link',
                 'imu_link'])

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('marathon_ros2_bringup'), 'launch/tiago',
            'nav2_tiago_launch.py')),
            launch_arguments={
              'autostart': 'true',
              'params_file': params_file,
              'cmd_vel_topic': '/cmd_vel'
            }.items())


    return launch.LaunchDescription([
        declare_params_file_cmd,
        rplidar_cmd,
        laserfilter_node,
        #realsense_node,
        kobuki_node,
        tf_kobuki2laser_node,
        tf_kobuki2imu_node,
        nav2_cmd
])
