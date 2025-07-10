#!/usr/bin/env python3

import rclpy
import tf_transformations
import csv

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from first_driver import *
from differential_drive import *


## ******************************
## CONTROLLER NODE CLASS
## ******************************

class ControllerNode(Node):

    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('controller_node')

        # Attributes
        self.timer_period_    = 0.1
        self.timer_read_data_ = 0.02
        
        self.vel_lineal_  = 0.0
        self.vel_angular_ = 0.0

        self.encoder_left_    = 0.0
        self.encoder_right_   = 0.0
        self.vel_left_wheel_  = 0.0
        self.vel_right_wheel_ = 0.0

        self.robot_traj_ = []


        # Publishers
        self.pub_joint_states_ = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odometry_     = self.create_publisher(Odometry, '/odom', 10)


        # Subscribers
        self.sub_velocities_ = self.create_subscription(Twist, '/cmd_vel', self.controller, 10)


        # Timers 
        self.tim_driver_data_  = self.create_timer(self.timer_read_data_, self.get_driver_data)
        self.tim_publish_data_ = self.create_timer(self.timer_period_, self.publish_data)


        # Instances
        self.first_driver_ = FirstDriver()
        self.dif_drive_    = DifferentialWheel(l=0.23, radius=0.0475)

    
    def controller(self, velocities_msg):
        """
        Calculates each motor speed from the robot's velocities message and sends them to the Esp32.

        Args:
            velocities_msg (twist): Robot velocities.
        """
        
        self.vel_lineal_ = velocities_msg.linear.x
        self.vel_angular_ = velocities_msg.angular.z

        self.vel_left_wheel_, self.vel_right_wheel_ = self.dif_drive_.get_motor_velocities(self.vel_lineal_, self.vel_angular_)

        self.first_driver_.send_motors_pow(self.vel_left_wheel_, self.vel_right_wheel_)


    def get_driver_data(self):
        """
        Reads the values of each encoder from the Esp32.
        """

        self.encoder_left_, self.encoder_right_ = self.first_driver_.read_motors_data()
        
        if self.encoder_left_ > 0:
            self.get_logger().info("Enc l: " + str(self.encoder_left_) + " | Enc r: " + str(self.encoder_right_))

    
    def update_odom(self):

        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()

        odom_msg.header.frame_id = "odom"

        odom_msg.pose.pose.position.x = float(self.dif_drive_.x)
        odom_msg.pose.pose.position.y = float(self.dif_drive_.y)
        odom_msg.pose.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.dif_drive_.theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x  = float(self.vel_lineal_)
        odom_msg.twist.twist.angular.z = float(self.vel_angular_)

        return odom_msg
    

    def update_joint_states(self):
        
        joint_states_msg = JointState()

        joint_states_msg.header.stamp = self.get_clock().now().to_msg()

        joint_states_msg.name     = ['left_wheel_joint', 'right_wheel_joint']
        joint_states_msg.position = [float(self.encoder_left_), float(self.encoder_right_)]
        joint_states_msg.velocity = [float(self.vel_left_wheel_), float(self.vel_right_wheel_)]

        return joint_states_msg

    
    def publish_data(self):

        odometry_msg    = self.update_odom()
        joint_state_msg = self.update_joint_states()

        self.pub_odometry_.publish(odometry_msg)
        self.pub_joint_states_.publish(joint_state_msg)

        wl = self.vel_left_wheel_ / self.dif_drive_.radius
        wr = self.vel_right_wheel_ / self.dif_drive_.radius

        self.dif_drive_.update_state(wl, wr, self.timer_period_)

        self.robot_traj_.append((
            float(self.dif_drive_.x),
            float(self.dif_drive_.y),
            float(self.dif_drive_.theta)
        ))

        


## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        print("Interrupció amb Ctrl+C — guardant trajectòria...")
    finally:
        # Escriure la trajectòria
        with open('trajectory_02.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'theta'])
            for point in controller_node.robot_traj_:
                writer.writerow(point)

        # Tancar correctament el node i ROS


if __name__ == "__main__":
    main()