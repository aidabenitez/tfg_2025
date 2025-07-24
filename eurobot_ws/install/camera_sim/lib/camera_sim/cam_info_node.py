#!/usr/bin/env python3

import rclpy
import yaml, sys, time

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


## ******************************
## CAMERA INFO NODE CLASS
## ******************************

class CamInfoNode(Node):

    def __init__(self):
        """
        Initializes all the attributes, publishers, subscribers and timers of the node.
        """

        super().__init__('cam_info_node')

        # Attributes
        self.timer_period_ = 0.1

        self.yaml_path_ = '/wolvi/src/camera_sim/config/cam_calibration.yaml'


        # Publishers
        self.pub_cam_info_ = self.create_publisher(CameraInfo, '/usb_cam/camera_info', 10)


        # Subscribers


        # Timers 
        self.tim_callback_ = self.create_timer(self.timer_period_, self.timer_callback)
        
    
    def prepare_message(self):

        with open(self.yaml_path_, 'r') as file:
            calib_data = yaml.safe_load(file)
        
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data['image_width']
        camera_info_msg.height = calib_data['image_height']
        camera_info_msg.distortion_model = calib_data['distortion_model']
        camera_info_msg.d = calib_data['distortion_coefficients']['data']
        camera_info_msg.k = calib_data['camera_matrix']['data']

        camera_info_msg.binning_x = 1
        camera_info_msg.binning_y = 1

        # ROI per defecte (no definit)
        camera_info_msg.roi.do_rectify = False

        camera_info_msg.header = Header()
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.header.frame_id = "camera_frame"

        return camera_info_msg

    
    def timer_callback(self):

        cam_msg = self.prepare_message()

        self.pub_cam_info_.publish(cam_msg)



## *************************
## MAIN CODE
## *************************

def main(args=None):
    rclpy.init(args=args)

    cam_info_node = CamInfoNode()

    rclpy.spin(cam_info_node)

    cam_info_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()