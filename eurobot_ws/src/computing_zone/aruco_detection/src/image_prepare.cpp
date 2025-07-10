#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <map>

using namespace cv;
using namespace std;
using namespace aruco;
using namespace std::chrono;



// *************************
// FUNCTIONS
// *************************

Mat parse_camera_matrix(istream& in) {
    /*
        Parses the camera matrix from the input stream and stores the values in a 3x3 matrix.
    
        Arguments:
            in (istream&): The input stream containing the camera matrix data.
    
        Returns:
            Mat: The 3x3 matrix containing the camera matrix values.
    */

    // Create an initialize the matrix and variables
    Mat mat(3, 3, CV_64F);
    string line;
    int row = 0;
    bool started = false;

    // Read the stream line by line
    while (getline(in, line)) {

        // Skip lines until we find the beginning of the matrix (indicated by a '[' character)
        if (!started && line.find('[') == string::npos) continue;
        started = true;

        // Remove brackets if present
        line.erase(remove(line.begin(), line.end(), '['), line.end());
        line.erase(remove(line.begin(), line.end(), ']'), line.end());

        // Use stringstream to extract numerical values from the cleaned line
        stringstream ss(line);
        double val;
        int col = 0;

        // Read values separated by spaces, commas, or semicolons
        while (ss >> val) {
            mat.at<double>(row, col++) = val;
            if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
        }
        
        row++;
    }

    return mat;
}

Mat parse_dist_coeffs(istream& in) {
    /*
        Parses the distortion coefficients from the input stream and stores the values in a 1x5 matrix.
    
        Arguments:
            in (istream&): The input stream containing the distortion coefficients data.
    
        Returns:
            Mat: The 1x5 matrix containing the distortion coefficients.
    */
    
    // Create an initialize the matrix and variables
    Mat dist_coeffs(1, 5, CV_64F);
    string line;

    // Read the stream line by line
    while (getline(in, line)) {

        // Find the positions of the opening and closing brackets
        size_t start = line.find('[');
        size_t end = line.find(']');

        // If both brackets are found in the line, extract the data between them
        if (start != string::npos && end != string::npos) {

            // Extract substring between brackets
            line = line.substr(start + 1, end - start - 1);
            stringstream ss(line);

            double val;
            int col = 0;

            // Read each number and assign it to the matrix
            while (ss >> val) {
                dist_coeffs.at<double>(0, col++) = val;
                if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
            }
        }
    }

    return dist_coeffs;
}



// *************************
// MAIN CODE
// *************************

int main() {


    // -- OPEN THE VIDEO --

    string video_path = "/wolvi/src/computing_zone/aruco_detection/assets/robot_demo/prova.avi";
    VideoCapture cap(video_path);

    // Obtenir l'amplada i alçada del vídeo original
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));

    // Crear el VideoWriter per escriure el vídeo de sortida
    string output_path = "/wolvi/src/computing_zone/aruco_detection/assets/robot_demo/video_result.avi";
    VideoWriter output_video(output_path, VideoWriter::fourcc('M','J','P','G'), 30, Size(frame_width, frame_height));

    if (!output_video.isOpened()) {
        cerr << "No s'ha pogut obrir el fitxer de sortida per escriure: " << output_path << endl;
        return -1;
    }


    if (!cap.isOpened()) {
        cerr << "Could not open video: " << video_path << endl;
        return -1;
    }

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


    // -- CAMERA POSE TRANSFORMATION --

    Mat frame;

    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            cout << "Video ended." << endl;
            break;
        }

        // -- ARUCO DETECTION --

        // Set the dictionary
        Ptr<Dictionary> opencv_dict = getPredefinedDictionary(DICT_4X4_100);

        // Declare result variables
        vector<int> ids;
        vector<vector<Point2f>> corners;

        // Detect arucos
        detectMarkers(frame, opencv_dict, corners, ids);

        // Calculate the corners center
        std::map<int, cv::Point2f> marker_centers;

        for (size_t i = 0; i < ids.size(); i++) {
            Point2f center(0, 0);
            for (const auto& point : corners[i]) { center += point; }  // Sum all the corners
            center *= 0.25f;  // Corners average

            // Store the center with its ID
            marker_centers[ids[i]] = center;
        }

        if (!ids.empty()) drawDetectedMarkers(frame, corners, ids);

        output_video.write(frame);

        imshow("Video reproduction", frame);

        // Wait the computed delay
        if (waitKey(delay) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}