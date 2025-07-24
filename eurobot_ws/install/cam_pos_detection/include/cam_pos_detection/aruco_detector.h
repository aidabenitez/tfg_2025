#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>


class ArucoDetector {

    public:

        ArucoDetector();
        
        void detect(const cv::Mat& image, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners);
        std::map<int, cv::Point2f> computeCenters(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners) const;


    private:

        cv::Ptr<cv::aruco::Dictionary> dictionary_;

};