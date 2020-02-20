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

    marathon_dir = get_package_share_directory('marathon_ros2_bringup')
    marathon_launch_dir = os.path.join(marathon_dir, 'launch')

    # Create the launch configuration variables
    next_wp = LaunchConfiguration('next_wp')

    autostart = LaunchConfiguration('autostart')
   
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_next_wp_cmd = DeclareLaunchArgument(
        'next_wp',
        default_value= '0',
        description='Next waypoint for the navigation system')

    log_node = launch_ros.actions.Node(
      package='marathon_log_nodes',
      node_executable='marathon_log_node',
    )

    wp_manager_node = launch_ros.actions.Node(
      package='marathon_ros2_wp_manager',
      node_executable='wp_manager_node',
      parameters=[
      {'next_wp': next_wp}]
    )

    topics_2_csv_node = launch_ros.actions.Node(
      package='marathon_ros2_csv',
      node_executable='topics_2_csv'
    )

    return launch.LaunchDescription([
        declare_next_wp_cmd,
        log_node,
        wp_manager_node,
        topics_2_csv_node
])
