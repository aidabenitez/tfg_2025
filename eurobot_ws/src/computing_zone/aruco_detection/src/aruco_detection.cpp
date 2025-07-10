#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace cv;
using namespace std;
using namespace aruco;


// *************************
// FUNCTIONS
// *************************

Mat parseCameraMatrix(istream& in) {
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
            cout << "VAL: " << val << endl;
            mat.at<double>(row, col++) = val;
            if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
        }
        
        row++;
    }

    return mat;
}

Mat parseDistCoeffs(istream& in) {
    /*
        Parses the distortion coefficients from the input stream and stores the values in a 1x5 matrix.
    
        Arguments:
            in (istream&): The input stream containing the distortion coefficients data.
    
        Returns:
            Mat: The 1x5 matrix containing the distortion coefficients.
    */
    
    // Create an initialize the matrix and variables
    Mat distCoeffs(1, 5, CV_64F);
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
                distCoeffs.at<double>(0, col++) = val;
                if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
            }
        }
    }

    return distCoeffs;
}



// *************************
// MAIN CODE
// *************************

int main() {
    
    // -- READING THE IMAGE -- 

    // Load test image
    Mat img = imread("/wolvi/src/computing_zone/aruco_detection/assets/playmat_02/photo_2.png");
    int h = img.rows;
    int w = img.cols;

    // Read the values of the camera calibration
    ifstream mtx_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/camera_matrix_output.txt");
    Mat mtx = parseCameraMatrix(mtx_file);
    mtx_file.close();

    ifstream dits_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/distortion_coefficients_output.txt");
    Mat dist = parseDistCoeffs(dits_file);
    dits_file.close();

    // Calculare the optimal camera matrix
    Mat newCameraMatrix; Rect roi;
    newCameraMatrix = getOptimalNewCameraMatrix(mtx, dist, Size(w, h), 1, Size(w, h), &roi);

    // Undistort the image
    Mat dst;
    undistort(img, dst, mtx, dist, newCameraMatrix);

    // Crop the image
    dst = dst(roi);

    if (dst.empty()) {
        cerr << "Error: Camera matrix or distortion coefficients are empty!" << endl;
        return -1;
    }


    // -- ARUCO DETECTION --

    // Set the dictionary
    Ptr<Dictionary> opencvDict = getPredefinedDictionary(DICT_4X4_100);

    // Declare result variables
    vector<int> ids;
    vector<vector<Point2f>> corners;

    // Detect arucos
    detectMarkers(dst, opencvDict, corners, ids);


    // -- ARUCO DRAWING --

    std::map<int, cv::Point2f> markerCenters;

    // Draw all the corners
    if (!ids.empty()) {
        drawDetectedMarkers(dst, corners, ids);
    } else {
        cout << "Not arucos detected." << endl;
    }

    // Show the image with the transformed (warped) perspective
    imshow("Aruco Image", dst);
    waitKey(0);

    destroyAllWindows();
    return 0;
}
