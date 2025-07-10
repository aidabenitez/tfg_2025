#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


## ******************************
## SIMPLE NAVIGATION NODE CLASS
## ******************************

class SimpleNavigationNode(Node):

    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('simple_navigation_node')

        # Attributes
        self.timer_period_ = 0.1

        self.state_ = 0  # 0 = forward; 1 = turn
        self.step_count_ = 0
        self.steps_per_move_ = int(1 / self.timer_period_)
        self.current_step_ = 0


        # Publishers
        self.pub_velocity_ = self.create_publisher(Twist, '/cmd_vel', 10)


        # Subscribers


        # Timers 
        self.tim_callback_  = self.create_timer(self.timer_period_, self.timer_callback)

    
    def timer_callback(self):

        if self.step_count_ >= 4:
            self.get_logger().info("Quadrat completat. Parant el robot.")
            stop_msg = Twist()
            self.pub_velocity_.publish(stop_msg)
            self.destroy_timer(self.timer_period_)

        else:
            velocity_msg = Twist()

            if self.state_ == 0:
                velocity_msg.linear.x = 0.2
                velocity_msg.angular.z = 0.0
            elif self.state_ == 1:
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 1.57

            self.pub_velocity_.publish(velocity_msg)

            self.current_step_ += 1

            if self.current_step_ >= self.steps_per_move_:
                self.current_step_ = 0
                self.state_ = (self.state_ + 1) % 2
                if self.state_ == 0:
                    self.step_count_ += 1


        



## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    simple_navigation_node = SimpleNavigationNode()

    rclpy.spin(simple_navigation_node)

    simple_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()