#include "rclcpp/rclcpp.hpp"
#include "cam_pos_detection/srv/get_camera_pose.hpp"

#include <memory>

class CameraPositionService : public rclcpp::Node
{

    public:
        CameraPositionService();

    private:
        // Attributes

        // Services
        rclcpp::Service<cam_pos_detection::srv::GetCameraPose>::SharedPtr service_;

        // Private methods
        // void reload_cam_pos(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);
};


CameraPositionService::CameraPositionService() : Node("camera_position_service"), pose_calculator_()
{
    // service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
    //     "add_two_ints",
    //     std::bind(&CameraPositionService::handle_add, this,
    //         std::placeholders::_1,
    //         std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Ready to add two ints.");
}

// void CameraPositionService::reload_cam_pos(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
// {
//     response->sum = request->a + request->b;

//     RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld", request->a, request->b);
//     RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", (long int)response->sum);
// }




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPositionService>());
  rclcpp::shutdown();
  return 0;
}
