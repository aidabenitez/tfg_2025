#include "cam_pos_detection/image_processor.h"

ImageProcessor::ImageProcessor() {}

void ImageProcessor::setMatrices(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    cameraMatrix_ = camera_matrix;
    distCoeffs_   = dist_coeffs;
}

cv::Mat ImageProcessor::undistortImage(const cv::Mat& distortedImg) {
    int w = distortedImg.cols;
    int h = distortedImg.rows;

    cv::Rect roi;

    newCameraMatrix_ = cv::getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, cv::Size(w, h), 1, cv::Size(w, h), &roi);
    cv::Mat undistorted;
    cv::undistort(distortedImg, undistorted, cameraMatrix_, distCoeffs_, newCameraMatrix_);

    undistorted = undistorted(roi);

    if (undistorted.empty()) {
        std::cerr << "Error: Camera matrix or distortion coefficients are empty!" << std::endl;
        return cv::Mat();
    }

    return undistorted;
}
