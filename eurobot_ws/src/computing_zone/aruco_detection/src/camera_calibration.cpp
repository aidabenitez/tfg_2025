#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <glob.h>
#include <fstream>

using namespace cv;
using namespace std;

vector<String> getImages(const string& pattern) {
    vector<String> filenames;
    glob(pattern, filenames, false);
    return filenames;
}

int main() {
    // Termination criteria
    TermCriteria criteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    // Prepare object points
    vector<Point3f> objp;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 7; j++) {
            objp.push_back(Point3f((float)j, (float)i, 0));
        }
    }

    vector<vector<Point3f>> objpoints;
    vector<vector<Point2f>> imgpoints;

    // Get list of calibration images
    vector<String> images = getImages("/wolvi/src/computing_zone/aruco_detection/assets/calibration_photos/*.png");

    Mat gray, img;
    Size patternSize(7, 6);

    for (size_t i = 0; i < images.size(); i++) {
        img = imread(images[i]);
        cvtColor(img, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, patternSize, corners);

        if (found) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), criteria);
            imgpoints.push_back(corners);
            objpoints.push_back(objp);

            drawChessboardCorners(img, patternSize, corners, found);
            imshow("Chessboard", img);
            waitKey(500);
        }
    }

    destroyAllWindows();

    // Calibrate the camera
    Mat mtx, dist, R, T;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objpoints, imgpoints, gray.size(), mtx, dist, rvecs, tvecs);

    // Save calibration parameters to different text files
    ofstream calibFileCM("/wolvi/src/computing_zone/aruco_detection/calibration_values/camera_matrix_output.txt");

    if (calibFileCM.is_open()) { calibFileCM << "Camera Matrix (mtx):\n" << mtx << "\n"; } 
    else { cerr << "Unable to open file for writing calibration data." << endl; }

    ofstream calibFileDC("/wolvi/src/computing_zone/aruco_detection/calibration_values/distortion_coefficients_output.txt");

    if (calibFileDC.is_open()) { calibFileDC << "Distortion Coefficients (dist):\n" << dist << "\n"; } 
    else { cerr << "Unable to open file for writing calibration data." << endl; }

    return 0;
}
