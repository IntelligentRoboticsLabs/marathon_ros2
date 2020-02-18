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
#include "std_msgs/msg/empty.h"

using namespace std::placeholders;

class WaypointManager: public rclcpp::Node
{
public:
  explicit WaypointManager()
  : Node("waypoint_manager"), goal_sended_(false), starting_(false)
  {   
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = 20.0;
    p.pose.position.y = 46.5;
    p.header.frame_id = "map";
    waypoints_[0] = p;

    p.pose.position.x = 27.8;
    p.pose.position.y = 57.1;
    waypoints_[1] = p;

    p.pose.position.x = 34.3;
    p.pose.position.y = 57.6;
    waypoints_[2] = p;
    
    p.pose.position.x = 31.8;
    p.pose.position.y = 63.8;
    waypoints_[3] = p;

    p.pose.position.x = 38.7;
    p.pose.position.y = 74.6;
    waypoints_[4] = p;

    p.pose.position.x = 26.5;
    p.pose.position.y = 63.0;
    waypoints_[5] = p;

    p.pose.position.x = 21.7;
    p.pose.position.y = 49.0;
    waypoints_[6] = p;

    p.pose.position.x = 10.3;
    p.pose.position.y = 51.4;
    waypoints_[7] = p;

    this->declare_parameter("next_wp");
    next_wp_ = this->get_parameter("next_wp").get_value<int>();

    start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/start_navigate",
      1,
      std::bind(&WaypointManager::start_cb, this, _1));  

  }

  void start_cb(const std_msgs::msg::Empty::SharedPtr msg)
  {
    starting_ = true;
  }

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt((pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
             (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }
  
  void navigate_to_pose(geometry_msgs::msg::PoseStamped goal_pose){
    //rclcpp::spin_some(node_->get_node_base_interface());

    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "NavigateToPose");
       
    bool is_action_server_ready = false;

    do {
      RCLCPP_WARN(this->get_logger(), "Waiting for action server");
      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_WARN(this->get_logger(), "Starting navigation");

    navigation_goal_.pose = goal_pose;
    dist_to_move = getDistance(goal_pose.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
      RCLCPP_WARN(this->get_logger(), "Navigation completed");
      feedback_ = 100.0;
      goal_sended_ = false;
      next_wp_++;
      if (next_wp_ == waypoints_.size())
      {
        next_wp_ = 0;
      }
    };

    goal_handle_future_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
    goal_sended_ = true;
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future_) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "send goal call failed :(");
      goal_sended_ = false;
      return;
    }
    
    navigation_goal_handle_ = goal_handle_future_.get();  
    if (!navigation_goal_handle_) {
      goal_sended_ = false;
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
  }

  void step()
  {
    if (!goal_sended_ && starting_)
    {
      RCLCPP_WARN(this->get_logger(), "navigate_to_pose");
      navigate_to_pose(waypoints_[next_wp_]);
    }
      
  }

private:
  
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  std::map<int, geometry_msgs::msg::PoseStamped> waypoints_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  double dist_to_move;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> goal_handle_future_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  int next_wp_;

  float feedback_;
  bool goal_sended_, starting_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointManager>();

  rclcpp::Rate loop_rate(1); 
  while (rclcpp::ok()) {
    node->step();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}