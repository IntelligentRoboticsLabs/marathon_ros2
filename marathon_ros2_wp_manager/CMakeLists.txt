cmake_minimum_required(VERSION 3.5)
project(marathon_ros2_wp_manager)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


set(dependencies
    rclcpp
    rclcpp_action
    nav2_msgs
)

add_executable(wp_manager_node src/wp_manager.cpp)
ament_target_dependencies(wp_manager_node ${dependencies})

#install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  wp_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
