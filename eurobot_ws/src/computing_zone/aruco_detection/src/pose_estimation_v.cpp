#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <chrono>

using namespace cv;
using namespace std;
using namespace aruco;

Mat parse_camera_matrix(istream& in) {
    Mat mat(3, 3, CV_64F);
    string line;
    int row = 0;
    bool started = false;
    while (getline(in, line)) {
        if (!started && line.find('[') == string::npos) continue;
        started = true;
        line.erase(remove(line.begin(), line.end(), '['), line.end());
        line.erase(remove(line.begin(), line.end(), ']'), line.end());
        stringstream ss(line);
        double val;
        int col = 0;
        while (ss >> val) {
            mat.at<double>(row, col++) = val;
            if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
        }
        row++;
    }
    return mat;
}

Mat parse_dist_coeffs(istream& in) {
    Mat dist_coeffs(1, 5, CV_64F);
    string line;
    while (getline(in, line)) {
        size_t start = line.find('[');
        size_t end = line.find(']');
        if (start != string::npos && end != string::npos) {
            line = line.substr(start + 1, end - start - 1);
            stringstream ss(line);
            double val;
            int col = 0;
            while (ss >> val) {
                dist_coeffs.at<double>(0, col++) = val;
                if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
            }
        }
    }
    return dist_coeffs;
}

int main() {
    ifstream mtx_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/camera_matrix_output.txt");
    Mat mtx = parse_camera_matrix(mtx_file);
    mtx_file.close();

    ifstream dist_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/distortion_coefficients_output.txt");
    Mat dist = parse_dist_coeffs(dist_file);
    dist_file.close();

    VideoCapture cap("/wolvi/src/computing_zone/aruco_detection/assets/robot_demo/video_output.avi");
    if (!cap.isOpened()) return -1;

    ofstream log_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/robot_positions.csv");
    log_file << "time,cam_x,cam_y\n";

    // Compute the number of frames
    int total_frames = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    if (total_frames <= 0) {
        cerr << "Could not get the number of frames" << endl;
        return -1;
    }

    // Desired video duration
    double desired_duration_sec = 45.0;

    // Delay per frame (ms)
    double delay_per_frame_ms = (desired_duration_sec / total_frames) * 1000.0;
    int delay = static_cast<int>(delay_per_frame_ms);

    double original_width = 1600.0, original_height = 1200.0;
    double new_width = cap.get(CAP_PROP_FRAME_WIDTH);
    double new_height = cap.get(CAP_PROP_FRAME_HEIGHT);
    mtx.at<double>(0, 0) *= new_width / original_width;
    mtx.at<double>(0, 2) *= new_width / original_width;
    mtx.at<double>(1, 1) *= new_height / original_height;
    mtx.at<double>(1, 2) *= new_height / original_height;

    Ptr<Dictionary> dict = getPredefinedDictionary(DICT_4X4_100);
    map<int, Point2f> marker_centers;
    set<int> board_ids = {20, 21, 22, 23};

    Mat frame;
    while (cap.read(frame)) {
        vector<int> ids;
        vector<vector<Point2f>> corners;
        detectMarkers(frame, dict, corners, ids);
        marker_centers.clear();

        for (size_t i = 0; i < ids.size(); i++) {
            Point2f center(0, 0);
            for (const auto& p : corners[i]) center += p;
            center *= 0.25f;
            marker_centers[ids[i]] = center;
        }

        if (all_of(board_ids.begin(), board_ids.end(), [&](int id) { return marker_centers.count(id); })) break;
    }

    map<int, Point3f> board_points_mm = {
        {20, {2400, 600, 0}}, {21, {600, 600, 0}}, {22, {2400, 1400, 0}}, {23, {600, 1400, 0}}
    };

    vector<Point2f> image_points;
    vector<Point3f> board_points;
    for (int id : board_ids) {
        image_points.push_back(marker_centers[id]);
        board_points.push_back(board_points_mm[id]);
    }

    Mat board_rvec, board_tvec;
    solvePnP(board_points, image_points, mtx, dist, board_rvec, board_tvec);
    Mat board_rot;
    Rodrigues(board_rvec, board_rot);
    Mat board_rot_t = board_rot.t();
    Mat camera_pos = -board_rot_t * board_tvec;

    vector<Point2f> world_points_2d = {
        {2400, 600}, {600, 600}, {2400, 1400}, {600, 1400}
    };

    Mat H = findHomography(image_points, world_points_2d);
    int offsetX = 500;
    int offsetY = 500;
    Mat T = (cv::Mat_<double>(3,3) <<
        1, 0, offsetX,
        0, 1, offsetY,
        0, 0, 1);
    Mat H_adjusted = T * H;
    Size outputSize(3000 + offsetX, 2000 + offsetY);
    Mat flipY = (cv::Mat_<double>(3,3) <<
        1,  0, 0,
        0, -1, outputSize.height,
        0,  0, 1);
    Mat H_final = flipY * H_adjusted;

    auto start_time = chrono::steady_clock::now();

    while (cap.read(frame)) {
        vector<int> ids;
        vector<vector<Point2f>> corners;
        detectMarkers(frame, dict, corners, ids);

        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] != 3) continue;

            float marker_length = 100.0f;
            vector<Point3f> obj_points = {
                {-marker_length/2, marker_length/2, 0}, {marker_length/2, marker_length/2, 0},
                {marker_length/2, -marker_length/2, 0}, {-marker_length/2, -marker_length/2, 0}
            };

            Mat robot_rvec, robot_tvec;
            solvePnP(obj_points, corners[i], mtx, dist, robot_rvec, robot_tvec);

            Mat robot_world = board_rot_t * robot_tvec + camera_pos;
            double robot_x = robot_world.at<double>(0);
            double robot_y = robot_world.at<double>(1);
            double robot_z = robot_world.at<double>(2);

            vector<Point3f> real_points = { Point3f(robot_x, robot_y, robot_z) };
            vector<Point2f> projected_points;
            projectPoints(real_points, board_rvec, board_tvec, mtx, dist, projected_points);

            vector<Point2f> input_pt = { projected_points[0] };
            vector<Point2f> projected_pt;
            perspectiveTransform(input_pt, projected_pt, H_final);

            cout << "Projected robot position on homography (pixels): X="
                 << projected_pt[0].x << ", Y=" << 2000 - projected_pt[0].y << endl;

            auto now = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(now - start_time).count();
            log_file << elapsed << "," << projected_pt[0].x << "," << 2000 - projected_pt[0].y << "\n";

            circle(frame, projected_pt[0], 6, Scalar(0, 255, 0), -1);
        }

        imshow("Projected Homography", frame);
        if (waitKey(delay) == 'q') break;
    }

    log_file.close();
    cap.release();
    destroyAllWindows();
    return 0;
}
