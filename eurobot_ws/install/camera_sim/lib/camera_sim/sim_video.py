#!/usr/bin/env python3

import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


## ******************************
## VIDEO SIMULATION NODE CLASS
## ******************************

class SimVideoNode(Node):

    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('sim_video_node')

        # Attributes
        self.file_path_ = '/wolvi/src/camera_sim/assets/video_output_01.avi'
        self.cap_       = cv2.VideoCapture(self.file_path_)
        self.bridge_    = CvBridge()


        # Publishers
        self.pub_image_ = self.create_publisher(Image, '/usb_cam/image_raw', 10)


        # Subscribers


        # Timers 
        
    
    def publish_video(self):

        while self.cap_.isOpened():
            ret, frame = self.cap_.read()
            
            if ret:
                self.pub_image_.publish(self.bridge_.cv2_to_imgmsg(frame, "bgr8"))
            else:
                self.cap_.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self.cap_.release()



## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    sim_video_node = SimVideoNode()

    sim_video_node.publish_video()

    sim_video_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()