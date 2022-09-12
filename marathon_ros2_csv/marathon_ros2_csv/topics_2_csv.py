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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float64, Empty, Int16
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math

import csv
from datetime import datetime

class Topics2csv(Node):
    def __init__(self, last_contexts=None):
        super().__init__('topics_2_csv')
        #self.distance_sub_ = self.create_subscription(
        #  Float64,
        #  "/marathon_ros2/distance",
        #  self.distance_cb, 1)

        amcl_qos_profile = QoSProfile(
        depth=1,
        # reliability=QoSDurabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        scan_qos_profile = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE)

        self.amcl_sub_ = self.create_subscription(
          PoseWithCovarianceStamped,
          "/amcl_pose",
          self.amcl_cb, 1)

        self.scan_sub_ = self.create_subscription(
          LaserScan,
          "/scan",
          self.scan_cb,
          scan_qos_profile)

        self.recoveries_sub_ = self.create_subscription(
          Int16,
          "/marathon_ros2/number_of_recoveries",
          self.recoveries_cb, 1)

        self.it_sub_ = self.create_subscription(
          Empty,
          "/marathon_ros2/iteration_finished",
          self.it_cb, 1)
        
        self.vel_sub_ = self.create_subscription(
          Twist,
          "/nav_vel",
          self.vel_cb, 1)
          
        self.distance_ = 0.0
        self.old_x_ = 0.0
        self.old_y_ = 0.0
        self.scan_min_ = 10.0
        self.n_recoveries_ = 0
        self.vel_ = Twist()

        #self.get_logger().info("DF_CLIENT: Ready!")
        self.fieldnames_ = ['time', 'distance', 'recoveries_executed', 'vel_x', 'vel_theta', 'scan_min']

        username = os.environ['USER']
        self.path = "/home/" + username + "/exp_data/data/"
        self.update_csv_file_name()

        with open(self.path + self.csv_filename , mode='w+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            writer.writeheader()
        timer_period = 1.0 # seconds
        timer = self.create_timer(timer_period, self.step)

    def destroy(self):
        super().destroy_node()

    def amcl_cb(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        meters = 0.0
        if self.old_x_ != 0.0 and self.old_y_ != 0.0:
            meters = self.calculate_distance(current_x, current_y, self.old_x_, self.old_y_)
            print ("-----------------------")
            print (meters)
            self.old_x_ = current_x
            self.old_y_ = current_y
            #miles = metersToMiles(meters_);
        else:
            self.old_x_ = current_x
            self.old_y_ = current_y
    
        self.distance_ = meters

    def scan_cb(self, msg):
        for measure in msg.ranges:
            if measure < self.scan_min_ and not math.isinf(measure):
                self.scan_min_ = measure
            
    def recoveries_cb(self, msg):
        self.n_recoveries_ = msg.data
    
    def it_cb(self, msg):
        self.update_csv_file_name()

    def vel_cb(self, msg):
        self.vel_ = msg

    def update_csv_file_name(self):
        filename_list = []
        for file in os.listdir(self.path):
            filename_list.append(int(file[:-4]))
        filename_list_sorted = sorted(filename_list)

        if len(filename_list_sorted) == 0:
            self.csv_filename = str(1) + ".csv"
        else: 
            self.last_file_number = filename_list_sorted[-1]
            self.csv_filename = str(self.last_file_number + 1) + ".csv"

    def calculate_distance(self, current_x, current_y, old_x, old_y):
        return math.sqrt((pow(current_x - old_x, 2) + pow(current_y - old_y, 2)))

    def step(self):
        with open(self.path + self.csv_filename, mode='a+') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames_)
            writer.writerow({
                'time': self.get_clock().now().to_msg().sec,
                'distance': self.distance_,
                'recoveries_executed': self.n_recoveries_,
                'vel_x': self.vel_.linear.x,
                'vel_theta':self.vel_.angular.z,
                'scan_min': self.scan_min_})
            self.distance_ = 0.0
            self.n_recoveries_ = 0
          

def main(args=None):
    rclpy.init(args=args)
    node = Topics2csv()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()