#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "cam_pos_detection/image_processor.h"
#include "cam_pos_detection/aruco_detector.h"
#include "cam_pos_detection/pose_calculator.h"


class RobotPositionNode : public rclcpp::Node
{

  public:
    RobotPositionNode();


  private:

    // Attributes
    ImageProcessor image_processor_;
    ArucoDetector aruco_detector_;
    PoseCalculator pose_calculator_;




    // Publishers


    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
    // rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cam_pose_;


    // Timers
    rclcpp::TimerBase::SharedPtr timer_;


    // Private methods
    void get_image(const sensor_msgs::msg::Image image_msg);
    void get_cam_info(const sensor_msgs::msg::CameraInfo image_msg);

    void timer_callback();
};


RobotPositionNode::RobotPositionNode() : Node("robot_position_node"), image_processor_(), aruco_detector_()
{
  sub_image_    = this->create_subscription<sensor_msgs::msg::Image>("/usb_cam/image_raw", 10, std::bind(&RobotPositionNode::get_image, this, std::placeholders::_1));
  sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/usb_cam/camera_info", 10, std::bind(&RobotPositionNode::get_cam_info, this, std::placeholders::_1));
  // sub_cam_pose_ = this->create_subscription<geometry_msgs::msg::Vector3>("/camera_poser", 10, std::bind(&RobotPositionNode::get_cam_pose, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotPositionNode::timer_callback, this));
}


void RobotPositionNode::get_image(const sensor_msgs::msg::Image image_msg)
{
  RCLCPP_INFO(this->get_logger(), "Image received: %u x %u", image_msg.width, image_msg.height);

  // 1. Read the image
  cv::Mat received_img = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  cv::Mat undistorted_img = image_processor_.undistortImage(received_img);

  cv::imshow("Received img", received_img);
  cv::imshow("Undistorted img", undistorted_img);
  cv::waitKey(1);

  // 2. Aruco detection
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

  aruco_detector_.detect(undistorted_img, ids, corners);

  std::map<int, cv::Point2f> marker_centers = aruco_detector_.computeCenters(ids, corners);


  // 3. Robot position
  // std::map<int, cv::Point3f> board_points_mm = {
  //     {20, {2400, 600, 0}},
  //     {21, {600, 600, 0}},
  //     {22, {2400, 1400, 0}},
  //     {23, {600, 1400, 0}}
  // };

  // pose_calculator_.setReferenceMarkers(board_points_mm);

  // pose_calculator_.computeBoardPose(marker_centers);

  // std::set<int> reference_robot_ids = {1};

  // std::vector<cv::Point2f> robot_points;
  // for (size_t i = 0; i < ids.size(); ++i) {
  //     if (ids[i] == 1) {
  //         robot_points = corners[i];
  //         break;
  //     }
  // }  

  // cv::Point3f out_position = cv::Point3f();
  // float out_yaw_rad = 0.0;
  // pose_calculator_.computeRobotPose(robot_points, 100.0, out_position, out_yaw_rad);
}

void RobotPositionNode::get_cam_info(const sensor_msgs::msg::CameraInfo cam_info_msg)
{
  RCLCPP_INFO(this->get_logger(), "Camera info: %u x %u", cam_info_msg.width, cam_info_msg.height);

  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(cam_info_msg.k.data())).clone();
  cv::Mat dist_coeffs   = cv::Mat(cam_info_msg.d.size(), 1, CV_64F, const_cast<double*>(cam_info_msg.d.data())).clone();

  image_processor_.setMatrices(camera_matrix, dist_coeffs);
  pose_calculator_.setMatrices(camera_matrix, dist_coeffs);
}



void RobotPositionNode::timer_callback()
{

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPositionNode>());
  rclcpp::shutdown();
  return 0;
}