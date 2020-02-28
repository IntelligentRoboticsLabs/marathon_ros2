# marathon_ros2

[![](https://img.youtube.com/vi/tXp8wFZr68M/0.jpg)](https://www.youtube.com/watch?v=tXp8wFZr68M&feature=youtu.be "Click to play on You Tube")

## Working components for the experiment

- [x] Tiago drivers in ROS
- [x] ros1_bridge
- [x] [cmd_vel_mux](https://github.com/IntelligentRoboticsLabs/nav2-TIAGo-support/tree/master/cmd_vel_mux) and [tf_static_resender])(https://github.com/IntelligentRoboticsLabs/nav2-TIAGo-support/tree/master/tf_static_resender) to correctly bridge all the required topics and tf ROS <-> ROS2
- [x] Navigation2
- [x] STVL
- [x] TEB
- [x] Adjust/optimize speeds and other parameters for Tiago
- [x] Tool to monitorize experiment (distance and time)
- [x] ROSbag configuration
- [x] Launcher to launch experiment complete

## Instructions to reproduce this experiment

1. Create a new workspace with the ROS2 sources. Follow [this](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/) instructions of the ROS2 wiki. 
Before compile the workspace, change the ros1_bridge for [this version](https://github.com/fmrico/ros1_bridge/tree/dedicated_bridges).
    - Is important to compile this workspace, compile first without the ros1_bridge:
        ```
        colcon build --symlink-install --packages-skip ros1_bridge
        ```
    - If the compilation conclude and everything is OK do:
        ```
        source /opt/ros/melodic/setup.bash
        colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
        ```
2. Create other workspace with the marathon_ros2 and the dependencies.
    ```
    vcs import src < marathon.repos
    ```
3. When the two workspaces are compiled you can start with the marathon...
4. Open 6 differents terminals and run:

    ```
    # Terminal 1
    source ~/ros2_sources_ws/install/setup.bash
    source /opt/ros/melodic/setup.bash
    ros2 run ros1_bridge tf_static_1_to_2
    ```
    ```
    # Terminal 2
    source ~/ros2_sources_ws/install/setup.bash
    source /opt/ros/melodic/setup.bash
    ros2 run ros1_bridge imu_1_to_2
    ```
    ```
    # Terminal 3
    source ~/ros2_sources_ws/install/setup.bash
    source /opt/ros/melodic/setup.bash
    ros2 run ros1_bridge scan_1_to_2
    ```
    ```
    # Terminal 4
    source ~/ros2_sources_ws/install/setup.bash
    source /opt/ros/melodic/setup.bash
    ros2 run ros1_bridge tf_1_to_2
    ```
    ```
    # Terminal 5
    source ~/ros2_sources_ws/install/setup.bash
    source /opt/ros/melodic/setup.bash
    ros2 run ros1_bridge pc2_1_to_2 
    ```
    ```
    # Terminal 6
    source ~/ros2_sources_ws/install/setup.bash
    source /opt/ros/melodic/setup.bash
    ros2 run ros1_bridge twist_2_to_1
    ```

5. Open two more terminals and run:

    ```
    # Terminal 7
    source ~/ros2_sources_ws/install/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch marathon_ros2_bringup nav2_tiago_launch.py
    ```
    ```
    # Terminal 8
    source ~/ros2_sources_ws/install/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 topic pub /start_navigate std_msgs/msg/Empty {}
    ```
