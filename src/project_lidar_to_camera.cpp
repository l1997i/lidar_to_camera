#include <iostream>
#include <numeric>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

bool loadPCDfromBin(string filename, pcl::PointCloud<pcl::PointXYZI> &ptcloud)
{
    // Load the actual pointcloud.
    // const size_t kMaxNumberOfPoints = 1e6; // From Readme for raw files.
    // ptcloud->clear();
    // ptcloud->reserve(kMaxNumberOfPoints);

    std::ifstream input(filename, std::ios::in | std::ios::binary);
    if (!input)
    {
        std::cout << "Could not open pointcloud file.\n";
        return false;
    }

    for (size_t i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZI point;
        //Open a container and write (x, y, z, i) one at a time, it's ok
        input.read((char *)&point.x, 3 * sizeof(float));
        input.read((char *)&point.intensity, sizeof(float));
        ptcloud.push_back(point);
    }
    input.close();
    return true;
}

void convertPCDtoBin(std::string &in_file, std::string &out_file)
{   //! [pcd->bin] NOT USED
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1) //* load the file
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return; // (-1);
    }

    std::ofstream myFile(out_file.c_str(), std::ios::out | std::ios::binary);
    for (int j = 0; j < cloud->size(); j++)
    {

        myFile.write((char *)&cloud->at(j).x, sizeof(cloud->at(j).x));
        myFile.write((char *)&cloud->at(j).y, sizeof(cloud->at(j).y));
        myFile.write((char *)&cloud->at(j).z, sizeof(cloud->at(j).z));
        myFile.write((char *)&cloud->at(j).intensity, sizeof(cloud->at(j).intensity));
    }
    myFile.close();
}

vector<string> getFiles(string get_dir)
{
    vector<string> files;
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(get_dir.c_str())) == NULL)
    {
        cout << get_dir.c_str();
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0
            || strcmp(ptr->d_name, "..") == 0
            || strcmp(ptr->d_name, "timestamp.txt") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) ///file
        //printf("d_name:%s/%s\n",basePath,ptr->d_name);
        {
            files.push_back(ptr->d_name);
        }
        else if (ptr->d_type == 10) ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if (ptr->d_type == 4) ///dir
        {
            files.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(files.begin(), files.end());
    return files;
}

vector<string> getPairName(string pairs_dir)
{
    vector<string> pairs;
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(pairs_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0
        || strcmp(ptr->d_name, "..") == 0
        || strcmp(ptr->d_name, "timestamp.txt") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8 || ptr->d_type == 10) ///file || link file
        //printf("d_name:%s/%s\n",basePath,ptr->d_name);
        {
            continue;
        }
        else if (ptr->d_type == 4) ///dir
        {
            pairs.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    sort(pairs.begin(), pairs.end());
    return pairs;
}

void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT)
{

    double f_x = 460.2672;
    double f_y = 458.3281;
    double b = 0.21;
    double c_x = 514.5355;
    double c_y = 269.3614;

    RT.at<double>(0, 0) = -0.0219;
    RT.at<double>(0, 1) = -0.9998;
    RT.at<double>(0, 2) = 0.0020;
    RT.at<double>(0, 3) = 0.3689;
    RT.at<double>(1, 0) = -0.0576;
    RT.at<double>(1, 1) = -7.468971579491779E-04;
    RT.at<double>(1, 2) = -0.9983;
    RT.at<double>(1, 3) = -0.1345;
    RT.at<double>(2, 0) = 0.9981;
    RT.at<double>(2, 1) = -0.0220;
    RT.at<double>(2, 2) = -0.0576;
    RT.at<double>(2, 3) = -0.3817;
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

    P_rect_00.at<double>(0, 0) = f_x;
    P_rect_00.at<double>(0, 1) = 0;
    P_rect_00.at<double>(0, 2) = c_x;
    P_rect_00.at<double>(0, 3) = -P_rect_00.at<double>(0, 0) * b;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = f_y;
    P_rect_00.at<double>(1, 2) = c_y;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;
}

void projectLidarToCamera2(string img_path, string pts_path, string depth_path, string bin_path)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(pts_path, *cloud) == -1) //* load the file
    // { //! [pcd->bin] NOT USED
    //     std::cout << "Couldn't read pointCloud file\n";
    // }

    if (!loadPCDfromBin(pts_path, *cloud))
    {
        std::cout << "Couldn't read pointCloud file\n";
    }

    // load image from file
    cv::Mat img = cv::imread(img_path);

    // load Lidar points from file
    std::vector<LidarPoint> lidarPoints;

    // store calibration data in OpenCV matrices
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type);        // rotation matrix and translation vector
    loadCalibrationData(P_rect_00, R_rect_00, RT);

    // project lidar points
    cv::Mat visImg = img.clone();
    cv::Mat overlay = visImg.clone();

    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (const auto &it : *cloud)
    {
        // filter the not needed points
        float MaxX = 60.0, maxY = 60.0, minZ = -2;
        if (it.x < 0.0 || it.x > MaxX)
        {
            continue;
        }

        // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
        X.at<double>(0, 0) = it.x;
        X.at<double>(1, 0) = it.y;
        X.at<double>(2, 0) = it.z;
        X.at<double>(3, 0) = 1;

        // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera.
        // Store the result in Y.
        Y = P_rect_00 * RT * X; // * R_rect_00
        // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        float val = it.x;
        float maxVal = MaxX;
        int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
        int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
        cv::circle(overlay, pt, 1.0, cv::Scalar(0, green, red), -1);
    }

    float opacity = 0.65;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);
    cv::imwrite(depth_path, visImg);
    // convertPCDtoBin(pts_path, bin_path); //! [pcd->bin] NOT USED
}

void makeDir(string dir)
{
    if (access(dir.c_str(), 0) == -1)
    {
        cout << dir << " is not existing" << endl;
        cout << "now make it" << endl;
        int flag = mkdir(dir.c_str(), 0777);
        if (flag == 0)
        {
            cout << "make successfully" << endl;
        }
        else
        {
            cout << "make errorly" << endl;
        }
    }

    else if (access(dir.c_str(), 0) == 0)
    {
        cout << dir << " exists" << endl;
        cout << "now delete it and remake it" << endl;
        int flag = rmdir(dir.c_str());
        mkdir(dir.c_str(), 0777);
        if (flag == 0)
        {
            cout << "delete it successfully" << endl;
        }
        else
        {
            cout << "delete it errorly" << endl;
        }
    }
}

void projectSpecificDir(string basePath, string pair_name)
{
    DIR *dir;
    struct dirent *ptr;
    string img_dir, pts_dir, depth_dir, bin_dir;
    string img_prefix = "image_02/";
    string pts_prefix = "bin/";
    string depth_prefix = "proj_depth_rgb/";
    string bin_prefix = "bin/"; //! [pcd->bin] NOT USED

    img_dir = basePath + '/' + pair_name + '/' + img_prefix;
    pts_dir = basePath + '/' + pair_name + '/' + pts_prefix;
    depth_dir = basePath + '/' + pair_name + '/' + depth_prefix;
    bin_dir = basePath + '/' + pair_name + '/' + bin_prefix;

    makeDir(depth_dir);
    makeDir(bin_dir);

    vector<string> img_list = getFiles(img_dir);
    vector<string> pts_list = getFiles(pts_dir);

    for (int i = 0; i < img_list.size(); i++)
    {
        string imgNo = img_list[i].substr(0, img_list[i].length() - 4);
        string ptsNo = pts_list[i].substr(0, pts_list[i].length() - 4);
        if (img_list[i] != ".DS_Store" && pts_list[i] != ".DS_Store")
        {
            string img_path = img_dir + imgNo + ".png";
            string pts_path = pts_dir + imgNo + ".bin";
            string depth_path = depth_dir + imgNo + ".png";
            string bin_path = bin_dir + imgNo + ".bin"; //! [pcd->bin] NOT USED
            projectLidarToCamera2(img_path, pts_path, depth_path, bin_path);
        }
    }
}

int main()
{
    char basePath[100];
    string pairsPath;
    memset(basePath, '\0', sizeof(basePath));
    getcwd(basePath, 999);
    pairsPath = basePath;
    pairsPath = pairsPath + "/../pairs";
    vector<string> pair_list = getPairName(pairsPath);

    for (auto pair : pair_list)
    {
        pair = "../pairs/" + pair;
        projectSpecificDir(basePath, pair);
    }
}