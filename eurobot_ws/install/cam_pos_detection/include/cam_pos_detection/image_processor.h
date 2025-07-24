#include <opencv2/opencv.hpp>


class ImageProcessor {

    public:

        ImageProcessor();
        
        void setMatrices(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);
        cv::Mat undistortImage(const cv::Mat& distortedImg);


    private:

        cv::Mat cameraMatrix_;
        cv::Mat distCoeffs_;
        cv::Mat newCameraMatrix_;

};