#include "cam_pos_detection/pose_calculator.h"

PoseCalculator::PoseCalculator() {}

void PoseCalculator::setMatrices(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    cameraMatrix_ = camera_matrix;
    distCoeffs_   = dist_coeffs;
}

void PoseCalculator::setReferenceMarkers(const std::map<int, cv::Point3f>& reference_world_coords) {

    referenceWorldPoints_ = reference_world_coords;

}

bool PoseCalculator::computeBoardPose(const std::map<int, cv::Point2f>& detected_centers) {

    std::set<int> reference_ids = {20, 21, 22, 23};

    std::map<int, cv::Point3f> board_points_mm = {
        {20, {2400, 600, 0}},
        {21, {600, 600, 0}},
        {22, {2400, 1400, 0}},
        {23, {600, 1400, 0}}
    };

    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> board_points;

    for (const auto& [id, center] : detected_centers) {
        if (reference_ids.count(id) && board_points_mm.count(id)) {
            image_points.push_back(center);
            board_points.push_back(board_points_mm[id]);
        }
    }

    if (image_points.size() < 4)
        return false; // No hi ha prou punts

    solvePnP(board_points, image_points, cameraMatrix_, distCoeffs_, board_rvec_, board_tvec_);
    cv::Mat board_rot;
    Rodrigues(board_rvec_, board_rot);
    board_rot_t_ = board_rot.t();

    cv::Mat board_trans = (cv::Mat_<double>(3, 1) << board_tvec_.at<double>(0), board_tvec_.at<double>(1), board_tvec_.at<double>(2));
    camera_position_ = -board_rot_t_ * board_trans;

    std::cout << "Camera position (mm): X=" << camera_position_.at<double>(0)
              << ", Y=" << camera_position_.at<double>(1)
              << ", Z=" << camera_position_.at<double>(2) << std::endl;

    return true;

}

bool PoseCalculator::computeRobotPose(const std::vector<cv::Point2f>& robot_corners, float marker_length, cv::Point3f& out_position, float& out_yaw_rad) {

    if (robot_corners.size() != 4) return false;

    std::vector<cv::Point3f> robot_obj_points = {
        {-marker_length / 2,  marker_length / 2, 0},
        { marker_length / 2,  marker_length / 2, 0},
        { marker_length / 2, -marker_length / 2, 0},
        {-marker_length / 2, -marker_length / 2, 0}
    };

    cv::Mat robot_rvec, robot_tvec;
    solvePnP(robot_obj_points, robot_corners, cameraMatrix_, distCoeffs_, robot_rvec, robot_tvec);

    cv::Mat robot_rot;
    Rodrigues(robot_rvec, robot_rot);

    cv::Mat robot_trans = (cv::Mat_<double>(3, 1) << robot_tvec.at<double>(0), robot_tvec.at<double>(1), robot_tvec.at<double>(2));
    cv::Mat p_board = board_rot_t_ * robot_trans + camera_position_;

    out_position = cv::Point3f(p_board.at<double>(0), p_board.at<double>(1), p_board.at<double>(2));

    // Calcul del yaw (angle respecte a Z, opcional)
    out_yaw_rad = atan2(robot_rot.at<double>(1, 0), robot_rot.at<double>(0, 0));

    std::cout << "Robot position (mm): X=" << out_position.x
              << ", Y=" << out_position.y
              << ", Z=" << out_position.z << std::endl;

    return true;

}
