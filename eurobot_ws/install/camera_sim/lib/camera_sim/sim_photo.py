#!/usr/bin/env python3

import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


## ******************************
## IMAGE SIMULATION NODE CLASS
## ******************************

class SimImageNode(Node):

    def __init__(self):
        """
        Initializes the node, loads the image and sets up a timer to publish it.
        """
        super().__init__('sim_image_node')

        # Load the image (només una vegada)
        self.image_path_ = '/wolvi/src/camera_sim/assets/frame.png'
        self.image_      = cv2.imread(self.image_path_)

        if self.image_ is None:
            self.get_logger().error(f"No s'ha pogut carregar la imatge: {self.image_path_}")
            return

        self.bridge_     = CvBridge()

        # Publisher
        self.pub_image_ = self.create_publisher(Image, '/usb_cam/image_raw', 10)

        # Timer per publicar la imatge repetidament (ex: cada 0.1 s)
        timer_period = 0.1  # segons
        self.timer_ = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        """
        Publica la imatge carregada cada vegada que salta el timer.
        """
        msg = self.bridge_.cv2_to_imgmsg(self.image_, encoding='bgr8')
        self.pub_image_.publish(msg)


## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    sim_image_node = SimImageNode()

    rclpy.spin(sim_image_node)

    sim_image_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
