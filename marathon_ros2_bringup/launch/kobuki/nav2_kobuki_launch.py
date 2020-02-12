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

    astra_node = launch_ros.actions.Node(
            package='astra_camera',
            node_executable='astra_camera_node',
            node_name='astra_camera_node',
            output='screen',
            parameters=[
              {'width': 1280},
              {'height': 1024},
              {'framerate': 30.0}])

    kobuki_node = launch_ros.actions.Node(
      package='turtlebot2_drivers',
      node_executable='kobuki_node',
      output='screen')
    
    depth_to_pointcloud2_node = launch_ros.actions.Node(
      package='depthimage_to_pointcloud2',
      node_executable='depthimage_to_pointcloud2_node',
      output='screen')

    tf_kobuki2laser_node = launch_ros.actions.Node(
      package='tf2_ros',
      node_executable='static_transform_publisher',
      output='screen',
      arguments=['0.0', '0.0', '0.40',
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
    tf_kobuki2camera_node = launch_ros.actions.Node(
      package='tf2_ros',
      node_executable='static_transform_publisher',
      output='screen',
      arguments=['0.12', '0.0', '0.22',
                 '0', '0', '0', '1',
                 'base_link',
                 'openni_color_optical_frame'])
    tf_kobuki2depth_node = launch_ros.actions.Node(
      package='tf2_ros',
      node_executable='static_transform_publisher',
      output='screen',
      arguments=['0.12', '0.0', '0.22',
                 '-0.5', '0.5', '-0.5', '0.5',
                 'base_link',
                 'openni_depth_optical_frame'])
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('marathon_ros2_bringup'), 'launch/tiago',
            'nav2_tiago_launch.py')),
            launch_arguments={
              'autostart': 'true',
              'params_file': params_file,
              'cmd_vel_topic': '/cmd_vel'
            }.items())

    log_node = launch_ros.actions.Node(
    package='marathon_log_nodes',
    node_executable='marathon_log_node')

    return launch.LaunchDescription([
        declare_params_file_cmd,
        rplidar_cmd,
        laserfilter_node,
        astra_node,
        depth_to_pointcloud2_node,
        kobuki_node,
        tf_kobuki2laser_node,
        tf_kobuki2imu_node,
        tf_kobuki2camera_node,
        tf_kobuki2depth_node,
        nav2_cmd,
        log_node
])
