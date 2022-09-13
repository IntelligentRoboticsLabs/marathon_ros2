/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2020, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Ginés jonatan.gines@urjc.es */

/* Mantainer: Jonatan Ginés jonatan.gines@urjc.es */


#include <math.h>
#include <iostream> 
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"


#include <tf2/LinearMath/Quaternion.h>
#include "tf2/convert.h"

using namespace std::placeholders;
using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

class WaypointManager: public rclcpp::Node
{
public:
  explicit WaypointManager(): Node("waypoint_manager"), goal_sended_(false), starting_(false) {   
    //waypoints_[0] = newWp(2.56, 50.6, 0.675);
    //waypoints_[1] = newWp(12.4365, 50.3792, 0.675);

    //waypoints_[0] = newWp(19.81, 46.06, 0.675);
    //waypoints_[1] = newWp(27.76, 55.39, 0.675);
    //waypoints_[2] = newWp(12.4365, 50.3792, 0.675 + M_PI/2);


    //waypoints_[0] = newWp(37.3, 55.4, 0.675 - M_PI/2);
    //waypoints_[1] = newWp(32, 63.7, 0.675 + M_PI/2);
    //waypoints_[2] = newWp(26.4, 58.2, 0.675 + M_PI);
    //waypoints_[3] = newWp(27.76, 55.39, 0.675);

    waypoints_[0] = newWp(12.4365, 50.3792, 0.675);
    waypoints_[1] = newWp(19.81, 46.06, 0.675);
    waypoints_[2] = newWp(27.76, 55.39, 0.675);
    waypoints_[3] = newWp(37.3, 55.4, 0.675 - M_PI/2);
    waypoints_[4] = newWp(32, 63.7, 0.675 + M_PI/2);
    waypoints_[5] = newWp(26.4, 58.2, 0.675 + M_PI);

    // waypoints_[0] = newWp(-0-65, -1.8, 0.675);
    // waypoints_[1] = newWp(0.85, -1.87, 0.675);
    // waypoints_[2] = newWp(1.82, -0.56, 0.675);
    // waypoints_[3] = newWp(1.86, 0.51, 0.675);
    // waypoints_[4] = newWp(0.62, 1.75, 0.675 - M_PI/2);
    // waypoints_[5] = newWp(-0.56, 1.80, 0.675 + M_PI/2);
    // waypoints_[6] = newWp(-2.0, 0.48, 0.675 + M_PI);


    declare_parameter<int>("next_wp", 0);
    get_parameter("next_wp", next_wp_);

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/start_navigate",
      1,
      std::bind(&WaypointManager::start_cb, this, _1));
    n_recoveries_pub_ = 
      create_publisher<std_msgs::msg::Int16>("/marathon_ros2/number_of_recoveries", rclcpp::QoS(1));
    it_finished_pub_ = 
      create_publisher<std_msgs::msg::Empty>("/marathon_ros2/iteration_finished", rclcpp::QoS(1));

    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "/navigate_to_pose");
    n_recoveries_ = 0;
  }

  geometry_msgs::msg::PoseStamped newWp(float x, float y, float yaw) {
    tf2::Quaternion q;
    geometry_msgs::msg::PoseStamped p;
    
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = y;
    q.setRPY(0, 0, yaw);
    q.normalize();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    return p;
  }

  void feedback_callback(
    NavigationGoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    distance_remaining_ = feedback->distance_remaining;
    n_recoveries_ = feedback->number_of_recoveries;
    //RCLCPP_WARN(get_logger(), "distance_remaining_ %f", distance_remaining_);
    if (distance_remaining_ < 0.3 && now() - goal_sended_stamp_ > rclcpp::Duration::from_seconds(3.0) && goal_sended_) {
      RCLCPP_WARN(get_logger(), "OK Next wp");
      next_wp_++;
      goal_sended_ = false;
      std_msgs::msg::Int16 msg;
      msg.data = n_recoveries_;
      n_recoveries_pub_->publish(msg);
      if (next_wp_ == waypoints_.size()) {
        next_wp_ = 0;
      } else if (next_wp_ == 1) {
        std_msgs::msg::Empty it_msg;
        it_finished_pub_->publish(it_msg);
      }
    }  
  }

  void result_callback( const NavigationGoalHandle::WrappedResult & result) {
    RCLCPP_WARN(get_logger(), "result_callback");
    goal_sended_ = false;
  }

  void start_cb(const std_msgs::msg::Empty::SharedPtr msg) {
    starting_ = true;
  }

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2) {
    return sqrt((pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
               (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  
  void navigate_to_pose(geometry_msgs::msg::PoseStamped goal_pose) {
    //rclcpp::spin_some(node_->get_node_base_interface());
    bool is_action_server_ready = false;

    do {
      RCLCPP_WARN(get_logger(), "Waiting for action server");
      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(1));
    } while (!is_action_server_ready);

    RCLCPP_WARN(get_logger(), "Starting navigation");
    
    goal_sended_stamp_ = now();
    goal_sended_ = true;

    navigation_goal_.pose = goal_pose;
    dist_to_move = getDistance(goal_pose.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&WaypointManager::result_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&WaypointManager::feedback_callback, this, _1, _2);
  
    goal_handle_future_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
    

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "send goal call failed :(");
      goal_sended_ = false;
      return;
    }
    
    navigation_goal_handle_ = goal_handle_future_.get();  
    if (!navigation_goal_handle_) {
      goal_sended_ = false;
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    }
  }

  void step()
  {
    if (!goal_sended_ && starting_)
    {
      //RCLCPP_WARN(this->get_logger(), "navigate_to_pose");
      navigate_to_pose(waypoints_[next_wp_]);
    }
      
  }

private:

  std::map<int, geometry_msgs::msg::PoseStamped> waypoints_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  double dist_to_move;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> goal_handle_future_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr n_recoveries_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr it_finished_pub_;
  int next_wp_, n_recoveries_;
  float distance_remaining_;
  bool goal_sended_, starting_;
  rclcpp::Time goal_sended_stamp_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointManager>();

  rclcpp::Rate loop_rate(5); 
  while (rclcpp::ok()) {
    node->step();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}