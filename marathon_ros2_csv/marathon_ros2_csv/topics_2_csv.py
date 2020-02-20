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
 
import os
import os.path

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
          "/marathon_ros2/distance",
          self.distance_cb, 1)
        self.rec_flag_sub_ = self.create_subscription(
          Empty,
          "/marathon_ros2/starting_recoveries",
          self.rec_flag_cb, 1)
        
        self.vel_sub_ = self.create_subscription(
          Twist,
          "/cmd_vel",
          self.vel_cb, 1)

        self.distance_ = 0.0
        self.rec_flag_ = ''
        self.vel_ = Twist()

        #self.get_logger().info("DF_CLIENT: Ready!")
        self.fieldnames_ = ['time', 'distance', 'recovery_behavior_executed', 'vel_x', 'vel_theta']

        username = os.environ['USER']
        self.path = "/home/" + username + "/marathon_data/"
        file_list = sorted(os.listdir(self.path))
        if len(file_list) == 0:
          self.csv_filename = str(1) + ".csv"
        else: 
          last_file = file_list[-1]
          self.last_file_number = int(last_file[:-4])
          self.csv_filename = str(self.last_file_number + 1) + ".csv"

        with open(self.path + self.csv_filename , mode='w+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            writer.writeheader()
        timer_period = 1.0 # seconds
        timer = self.create_timer(timer_period, self.step)

    def destroy(self):
        super().destroy_node()

    def distance_cb(self, msg):
        self.distance_ = msg.data
    
    def rec_flag_cb(self, msg):
        self.rec_flag_ = '1'

    def vel_cb(self, msg):
        self.vel_ = msg
  
    def step(self):
        with open(self.path + self.csv_filename, mode='a+') as csv_file:
          writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
          writer.writerow({
              'time': self.get_clock().now().to_msg().sec,
              'distance': self.distance_,
              'recovery_behavior_executed': self.rec_flag_,
              'vel_x': self.vel_.linear.x,
              'vel_theta':self.vel_.angular.z})
          self.rec_flag_ = ''
          self.distance_ = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = Topics2csv()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()