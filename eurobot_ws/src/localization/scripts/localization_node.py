#!/usr/bin/env python3

import odometry_interface, ekf_filter
import numpy as np

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry


## ******************************
## LOCALIZATION NODE CLASS
## ******************************

class LocalizationNode(Node):

    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('localization_node')
        
        # Attributes
        self.timer_period_ = 0.1
        
        self.vel_lineal_  = 0
        self.vel_angular_ = 0
        self.dif_time_    = 0

        self.last_time_ = 0


        # Publishers
        self.pub_robot_pose_ = self.create_publisher(Pose2D, '/ekf/robot_pose', 10)


        # Subscribers
        self.sub_odom_    = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)     
        self.sub_cam_pos_ = self.create_subscription(Pose2D, '/camera_pos', self.cam_pos_callback, 10)


        # Timers
        self.tim_localization_ = self.create_timer(self.timer_period_, self.locate_robot)


        # Instances
        self.obj_odometry_ = odometry_interface()
        self.ekf_filter_   = ekf_filter()


    def odom_callback(self, odom_msg):

        vel_lineal  = odom_msg.twist.linear.x
        vel_angular = odom_msg.twist.angular.z

        current_time = self.get_clock().now().nanoseconds / 1e9
        dif_time     = current_time - self.last_time_

        self.last_time_ = current_time

        self.ekf_filter_.prediction(dif_time, np.array(vel_lineal, vel_angular))
        self.ekf_filter_.update_prediction()


    def cam_pos_callback(self, cam_pos_msg):

        z = np.array([[cam_pos_msg.x], 
                      [cam_pos_msg.y]])
        H = np.array([[1, 0, 0], 
                      [0, 1, 0]])
        
        self.ekf_filter_.update(z, H, self.ekf_filter_.R)


    def locate_robot(self):

        robot_position = self.ekf_filter_.get_current_state()

        pose_msg = Pose2D()
        pose_msg.x = robot_position[0, 0]
        pose_msg.y = robot_position[1, 0]
        pose_msg.theta = robot_position[2, 0]

        self.pub_robot_pose_.publish(pose_msg)



## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    localization_node = LocalizationNode()

    rclpy.spin(localization_node)

    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()