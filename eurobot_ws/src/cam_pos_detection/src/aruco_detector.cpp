#include "cam_pos_detection/aruco_detector.h"

ArucoDetector::ArucoDetector() {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
}

void ArucoDetector::detect(const cv::Mat& image, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners) {

    detectMarkers(image, dictionary_, corners, ids);

}

std::map<int, cv::Point2f> ArucoDetector::computeCenters(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners) const {

    std::map<int, cv::Point2f> marker_centers;

    for (size_t i = 0; i < ids.size(); i++) {
        cv::Point2f center(0, 0);
        for (const auto& point : corners[i]) { center += point; }  // Sum all the corners
        center *= 0.25f;  // Corners average

        // Store the center with its ID
        marker_centers[ids[i]] = center;

        std::cout << "ID: " << ids[i] << " - Center: " << center << std::endl;
    }

    return marker_centers;

}
