#include <opencv2/opencv.hpp>
#include <map>

class PoseCalculator {

    public:

        PoseCalculator();

        void setMatrices(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
        void setReferenceMarkers(const std::map<int, cv::Point3f>& reference_world_coords);
        bool computeBoardPose(const std::map<int, cv::Point2f>& detected_centers);
        bool computeRobotPose(const std::vector<cv::Point2f>& robot_corners, float marker_length, cv::Point3f& out_position, float& out_yaw_rad);


    private:

        cv::Mat cameraMatrix_, distCoeffs_;
        
        std::map<int, cv::Point3f> referenceWorldPoints_;

        cv::Mat board_rvec_, board_tvec_;
        cv::Mat board_rot_t_, camera_position_;
};