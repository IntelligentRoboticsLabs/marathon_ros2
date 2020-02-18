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

/* Author: Lorena Bajo Rebollo lorena.bajo@urjc.es */

/* Mantainer: Lorena Bajo Rebollo lorena.bajo@urjc.es */


#include <math.h>
#include <iostream> 
#include <memory>
#include <string>
#include <map>
#include <math.h>
#include <iostream>

#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

#include "tf2_ros/transform_listener.h"

#include "builtin_interfaces/msg/time.hpp"



#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace std::placeholders;
using namespace std::chrono_literals;

class MarathonLogNode : public rclcpp::Node
{
public:
  MarathonLogNode()
  : Node("marathon_log_node")
  {
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::QoS(10), std::bind(&MarathonLogNode::poseAMCLCallback, this, _1)); 
    distance_total_pub_ = create_publisher<std_msgs::msg::Float64>("/marathon_ros2/total_distance", rclcpp::QoS(10));
    time_pub_ = create_publisher<builtin_interfaces::msg::Time>("/marathon_ros2/time", rclcpp::QoS(10));
    total_distance_ = 0.0;

    this->declare_parameter("total_distance_sum");
    total_distance_ = this->get_parameter("total_distance_sum").get_value<float>();
    meters_ = 0.0;
  }


  void calculateTimeNavigating()
  {
    rclcpp::Time current_time_ = now();
    rclcpp::Time time_navigating_;

    double secs = (current_time_ - started_time_).seconds();
    double nanosecs = (current_time_ - started_time_).nanoseconds();

    builtin_interfaces::msg::Time msg_time;

    msg_time.sec = secs;
    msg_time.nanosec = nanosecs;

    time_pub_->publish(msg_time);

  }

  float calculateDistance(float current_x, float current_y, float old_x, float old_y)
  {
    return sqrt((pow(current_x - old_x, 2) + pow(current_y - old_y, 2)));
  }

  float metersToMiles(float meters)
  {
    return meters*0.000621371;
  }

  void setOldposition(int current_x, int current_y)
  {
      old_x = current_x;
      old_y = current_y;
  }
  
  void poseAMCLCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) //const
  {
    float current_x, current_y;

    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    meters_ = calculateDistance(current_x, current_y, old_x, old_y);
    setOldposition(current_x, current_y);
    float miles = metersToMiles(meters_);

    total_distance_ += miles;

    std_msgs::msg::Float64 total_dist;
    total_dist.data = total_distance_;

    distance_total_pub_->publish(total_dist);

    meters_ = 0.0;
  
  }

  void step()
  {
    calculateTimeNavigating();
  }

  void firstTime(){
    started_time_= now();
  }


protected:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_total_pub_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
    
  float old_x, old_y;
  float meters_;
  float total_distance_;

  rclcpp::Time started_time_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MarathonLogNode>();

  node->firstTime();

  rclcpp::Rate loop_rate(1); 
  while (rclcpp::ok()) {
    node->step();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}