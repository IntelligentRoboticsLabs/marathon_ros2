# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Twist

import csv
from datetime import datetime

class Topics2csv(Node):
    def __init__(self, last_contexts=None):
        super().__init__('topics_2_csv')
        self.distance_sub_ = self.create_subscription(
          Float64,
          "/marathon_ros2/total_distance",
          self.distance_cb, 1)
        self.rec_flag_sub_ = self.create_subscription(
          Empty,
          "/marathon_ros2/starting_recoveries",
          self.rec_flag_cb, 1)
        
        self.vel_sub_ = self.create_subscription(
          Twist,
          "/cmd_vel",
          self.vel_cb, 1)

        self.total_distance_ = 0.0
        self.rec_flag_ = ''
        self.vel_ = Twist()

        #self.get_logger().info("DF_CLIENT: Ready!")
        self.fieldnames_ = ['time', 'distance', 'recovery_behavior_executed', 'vel_x', 'vel_theta']
        self.now_str_ = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        with open("topics_to_csv_" + self.now_str_ + ".csv" , mode='w+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            writer.writeheader()
        timer_period = 0.2 # seconds
        timer = self.create_timer(timer_period, self.step)

    def destroy(self):
        super().destroy_node()

    def distance_cb(self, msg):
        self.total_distance_ = msg.data
    
    def rec_flag_cb(self, msg):
        self.rec_flag_ = '1'

    def vel_cb(self, msg):
        self.vel_ = msg
  
    def step(self):
        with open("topics_to_csv_" + self.now_str_ + ".csv", mode='a+') as csv_file:
          writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
          writer.writerow({
              'time': self.get_clock().now().to_msg().sec,
              'distance': self.total_distance_,
              'recovery_behavior_executed': self.rec_flag_,
              'vel_x': self.vel_.linear.x,
              'vel_theta':self.vel_.angular.z})
          self.rec_flag_ = ''

def main(args=None):
    rclpy.init(args=args)
    node = Topics2csv()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()