#include <iostream>
#include <numeric>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT) {
    RT.at<double>(0, 0) = 0.0080;
    RT.at<double>(0, 1) = -1.000;
    RT.at<double>(0, 2) = 0.0018;
    RT.at<double>(0, 3) = 0.0698;
    RT.at<double>(1, 0) = -0.0488;
    RT.at<double>(1, 1) = -0.0022;
    RT.at<double>(1, 2) = -0.9988;
    RT.at<double>(1, 3) = -0.1561;
    RT.at<double>(2, 0) = 0.9988;
    RT.at<double>(2, 1) = 0.0079;
    RT.at<double>(2, 2) = -0.0488;
    RT.at<double>(2, 3) = -0.3609;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;

    R_rect_00.at<double>(0, 0) = 9.999239e-01;
    R_rect_00.at<double>(0, 1) = 9.837760e-03;
    R_rect_00.at<double>(0, 2) = -7.445048e-03;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = -9.869795e-03;
    R_rect_00.at<double>(1, 1) = 9.999421e-01;
    R_rect_00.at<double>(1, 2) = -4.278459e-03;
    R_rect_00.at<double>(1, 3) = 0.0;
    R_rect_00.at<double>(2, 0) = 7.402527e-03;
    R_rect_00.at<double>(2, 1) = 4.351614e-03;
    R_rect_00.at<double>(2, 2) = 9.999631e-01;
    R_rect_00.at<double>(2, 3) = 0.0;
    R_rect_00.at<double>(3, 0) = 0;
    R_rect_00.at<double>(3, 1) = 0;
    R_rect_00.at<double>(3, 2) = 0;
    R_rect_00.at<double>(3, 3) = 1;

    P_rect_00.at<double>(0, 0) = 432.2450;
    P_rect_00.at<double>(0, 1) = 0;
    P_rect_00.at<double>(0, 2) = 510.567850;
    P_rect_00.at<double>(0, 3) = -P_rect_00.at<double>(0, 0) * 0.21;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = 430.843025;
    P_rect_00.at<double>(1, 2) = 269.566049;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;

}

void projectLidarToCamera2() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/pcl/lidar_to_camera/pairs/0009.pcd", *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read pointCloud file\n";
    }
    // load image from file
    cv::Mat img = cv::imread("/home/pcl/lidar_to_camera/pairs/0009.png");

    // load Lidar points from file
    std::vector<LidarPoint> lidarPoints;
    // readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    // store calibration data in OpenCV matrices
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationData(P_rect_00, R_rect_00, RT);

    // project lidar points
    cv::Mat visImg = img.clone();
    cv::Mat overlay = visImg.clone();

    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (const auto& it: *cloud) {
        // filter the not needed points
        float MaxX = 25.0, maxY = 6.0, minZ = -1.40;
        if (it.x > MaxX ||it.x < 0.0 || abs(it.y) > maxY || it.z < minZ) {
            continue;
        }

        // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
        X.at<double>(0, 0) = it.x;
        X.at<double>(1, 0) = it.y;
        X.at<double>(2, 0) = it.z;
        X.at<double>(3, 0) = 1;

        // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera. 
        // Store the result in Y.
        Y = P_rect_00 * RT * X;  // * R_rect_00
        // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        float val = it.x;
        float maxVal = 20.0;
        int red = min(255, (int) (255 * abs((val - maxVal) / maxVal)));
        int green = min(255, (int) (255 * (1 - abs((val - maxVal) / maxVal))));
        cv::circle(overlay, pt, 1, cv::Scalar(0, green, red), -1);
    }

    float opacity = 0.6;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

    string windowName = "LiDAR data on image overlay";
//     cv::namedWindow(windowName, 3);
//     cv::imshow(windowName, visImg);
//     cv::waitKey(0); // wait for key to be pressed
    cv::imwrite("/home/pcl/lidar_to_camera/build/out.jpg",visImg);
}

int main() {
    projectLidarToCamera2();
}
