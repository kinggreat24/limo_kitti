// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames,
                vector<string> &vstrSemanticFilenames, vector<string> &vstrLidarFilenames, vector<double> &vTimestamps);

void LoadGtPoses(const std::string &gt_file, std::vector<Eigen::Matrix4d> &gtPoses, bool convert_to_origin = false);

int ReadPointCloud(const std::string &file, pcl::PointCloud<pcl::PointXYZI>::Ptr outpointcloud, bool isBinary);

void PublishTF(const Eigen::Matrix4d &T_cam_lidar);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle nh, nh_private("~");
    std::string dataset_folder;
    int sequence_number = 0;
    nh_private.getParam("dataset_folder", dataset_folder); //数据集路径
    nh_private.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';

    int publish_delay = 1;
    nh_private.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
    ros::Publisher pubOdomGT = nh.advertise<nav_msgs::Odometry>("/odometry_gt", 1);
    ros::Publisher pubPathGT = nh.advertise<nav_msgs::Path>("/path_gt", 1);
    ros::Publisher pubCameraInfo = nh.advertise<sensor_msgs::CameraInfo>("/image_left_camera_info", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 1);
    image_transport::Publisher pub_image_semantic = it.advertise("/image_semantic", 1);

    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/odom";
    odomGT.child_frame_id = "/base_link";

    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/odom";

    int imWidth, imHeight;
    Eigen::Matrix3d mK = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d mT_cam_lidar = Eigen::Matrix4d::Identity();
    sensor_msgs::CameraInfo camera_info;

    if (sequence_number >= 0 && sequence_number <= 2)
    {
        imWidth = 1241;
        imHeight = 376;
        // KITTI 00-02
        mK << 718.856, 0, 607.1928,
            0, 718.856, 185.2157,
            0, 0, 1;

        mT_cam_lidar << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
            -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
            9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
            0, 0, 0, 1;
    }
    else if (3 == sequence_number)
    {
        imWidth = 1242;
        imHeight = 375;
        // KITTI 03
        mK << 721.5377, 0, 609.5593,
            0, 721.5377, 172.8540,
            0, 0, 1;

        mT_cam_lidar << 0.0002347736981471, -0.9999441545438, -0.01056347781105, -0.002796816941295,
            0.01044940741659, 0.01056535364138, -0.9998895741176, -0.07510879138296,
            0.9999453885620, 0.0001243653783865, 0.01045130299567, -0.2721327964059,
            0, 0, 0, 1;
    }
    else if (sequence_number >= 4)
    {
        imWidth = 1226;
        imHeight = 370;
        // KITTI 04-12
        mK << 718.856, 0, 607.1928,
            0, 718.856, 185.2157,
            0, 0, 1;

        mT_cam_lidar << -0.001857739385241, -0.9999659513510, -0.008039975204516, -0.004784029760483,
            -0.006481465826011, 0.008051860151134, -0.9999466081774, -0.07337429464231,
            0.9999773098287, -0.001805528627661, -0.006496203536139, -0.3339968064433,
            0, 0, 0, 1;
    }

    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    camera_info.width = imWidth;
    camera_info.height = imHeight;
    camera_info.distortion_model = "pinhole";
    for (int i = 0; i < 9; i++)
        camera_info.K[i] = mK(i / 3, i % 3);
    camera_info.roi.do_rectify = false;

    // Retrieve paths to images
    vector<string> vstrImageFilenamesLeft;
    vector<string> vstrImageFilenamesSemantic;
    vector<string> vstrImageFilenamesLidar;
    vector<double> vTimestamps;

    char string_buffer[256] = {0};
    sprintf(string_buffer, "%s/sequences/%02d", dataset_folder.c_str(), sequence_number);
    string strAssociationFilename = std::string(string_buffer);
    LoadImages(strAssociationFilename, vstrImageFilenamesLeft, vstrImageFilenamesSemantic, vstrImageFilenamesLidar, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesLeft.size();
    if (vstrImageFilenamesLeft.empty())
    {
        cerr << endl
             << "No images found in provided path." << endl;
        return 1;
    }
    else if (vstrImageFilenamesLidar.size() != vstrImageFilenamesLeft.size())
    {
        cerr << endl
             << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    ROS_INFO("nImages: %d", nImages);

    std::vector<Eigen::Matrix4d> mvGtPoses;
    sprintf(string_buffer, "%s/poses/%02d.txt", dataset_folder.c_str(), sequence_number);
    string strGtPoseFiles = std::string(string_buffer);
    LoadGtPoses(strGtPoseFiles, mvGtPoses);

    // ros::Duration(10.0).sleep();

    cv::Mat imGray, imSemantic;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pLidar(new pcl::PointCloud<pcl::PointXYZI>());
    int ni = 0;
    ROS_INFO("publish_delay: %d", publish_delay);
    ros::Rate rate(10.0 / publish_delay);
    while (ros::ok())
    {
        if (ni >= nImages)
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        PublishTF(mT_cam_lidar);

        // ROS_WARN("Advertise Frame: %d", ni);
        // Read sensor data from file
        imGray = cv::imread(vstrImageFilenamesLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imSemantic = cv::imread(vstrImageFilenamesSemantic[ni], CV_LOAD_IMAGE_UNCHANGED);
        ReadPointCloud(vstrImageFilenamesLidar[ni], pLidar, true);
        double timestamp = vTimestamps[ni];

        if (imGray.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << vstrImageFilenamesLeft[ni] << endl;
            return 1;
        }

        ros::Time ctime = ros::Time::now();

        //发布图像数据
        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imGray).toImageMsg();
        image_left_msg->header.frame_id = "image_left";
        image_left_msg->header.stamp = ctime;
        sensor_msgs::ImagePtr image_semantic_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imSemantic).toImageMsg();
        image_semantic_msg->header.frame_id = "image_semantic";
        image_semantic_msg->header.stamp = ctime;
        pub_image_left.publish(image_left_msg);
        pub_image_semantic.publish(image_semantic_msg);

        pubCameraInfo.publish(camera_info);

        // 发布激光数据
        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(*pLidar, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ctime;
        laser_cloud_msg.header.frame_id = "sensor/velodyne";
        pub_laser_cloud.publish(laser_cloud_msg);

        // 发布外参关系
        Eigen::Quaterniond q_w_i(mvGtPoses.at(ni).topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * mvGtPoses.at(ni).topRightCorner<3, 1>();

        // 发布真值轨迹与odom
        odomGT.header.stamp = ctime;
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);

        ni++;
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Done \n";
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames,
                vector<string> &vstrSemanticFilenames, vector<string> &vstrLidarFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixLidar = strPathToSequence + "/velodyne/";
    string strPrefixSemantic = strPathToSequence + "/label/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
    vstrSemanticFilenames.resize(nTimes);
    vstrLidarFilenames.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
        vstrSemanticFilenames[i] = strPrefixSemantic + ss.str() + ".png";
        vstrLidarFilenames[i] = strPrefixLidar + ss.str() + ".bin";
    }
}

void LoadGtPoses(const std::string &gt_file, std::vector<Eigen::Matrix4d> &gtPoses, bool convert_to_origin)
{
    FILE *fp = fopen(gt_file.c_str(), "r");
    if (!fp)
        return;

    Eigen::Matrix4d pose_origin = Eigen::Matrix4d::Identity();
    while (!feof(fp))
    {
        Eigen::Matrix<double, 3, 4> P;
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0, 0), &P(0, 1), &P(0, 2), &P(0, 3),
                   &P(1, 0), &P(1, 1), &P(1, 2), &P(1, 3),
                   &P(2, 0), &P(2, 1), &P(2, 2), &P(2, 3)) == 12)
        {
            if (convert_to_origin)
            {
                if (gtPoses.size() > 0)
                {
                    //转换到以第一帧相机为原点的坐标系中
                    Eigen::Matrix4d raw_pose = Eigen::Matrix4d::Identity();
                    raw_pose.block<3, 4>(0, 0) = P;
                    Eigen::Matrix4d converted_pose = pose_origin.inverse() * raw_pose;
                    gtPoses.push_back(converted_pose);
                }
                else
                {
                    gtPoses.push_back(pose_origin);

                    //第一帧姿态
                    pose_origin.block<3, 4>(0, 0) = P;
                }
            }
            else
            {
                Eigen::Matrix4d raw_pose = Eigen::Matrix4d::Identity();
                raw_pose.block<3, 4>(0, 0) = P;
                gtPoses.push_back(raw_pose);
            }
        }
    }
    fclose(fp);
}

int ReadPointCloud(const std::string &file, pcl::PointCloud<pcl::PointXYZI>::Ptr outpointcloud, bool isBinary)
{
    // pcl::PointCloud<PointType>::Ptr curPointCloud(new pcl::PointCloud<PointType>());
    outpointcloud->points.clear();
    if (isBinary)
    {
        // load point cloud
        std::fstream input(file.c_str(), std::ios::in | std::ios::binary);
        if (!input.good())
        {
            std::cerr << "Could not read file: " << file << std::endl;
            exit(EXIT_FAILURE);
        }

        for (int i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI point;
            input.read((char *)&point.x, 3 * sizeof(float));
            input.read((char *)&point.intensity, sizeof(float));

            //remove all points behind image plane (approximation)
            /*if (point.x < mMinDepth)
                continue;*/

            // float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            // if (dist < 2)
            //     continue;

            outpointcloud->points.push_back(point);
        }
    }
    else
    {
        if (-1 == pcl::io::loadPCDFile<pcl::PointXYZI>(file, *outpointcloud))
        {
            std::cerr << "Could not read file: " << file << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    outpointcloud->height = 1;
    outpointcloud->width = outpointcloud->points.size();

    // SavePointCloudPly("/home/bingo/pc/loam/pointcloud/pc.ply",outpointcloud);
    return outpointcloud->points.size();
}

void PublishTF(const Eigen::Matrix4d &T_cam_lidar)
{
    Eigen::Vector3d translation = T_cam_lidar.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = T_cam_lidar.topLeftCorner(3, 3);
    Eigen::Quaterniond q_cam_lidar = Eigen::Quaterniond(rotation);

    // std::string sourceFrame = "sensor/velodyne";
    // std::string targetFrame = "sensor/camera";

    std::string targetFrame = "sensor/velodyne";
    std::string sourceFrame = "sensor/camera";

    static tf::TransformBroadcaster odometery_tf_publisher;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = sourceFrame;
    odom_trans.child_frame_id = targetFrame;

    odom_trans.transform.translation.x = translation[0];
    odom_trans.transform.translation.y = translation[1];
    odom_trans.transform.translation.z = translation[2];
    odom_trans.transform.rotation.x = q_cam_lidar.x();
    odom_trans.transform.rotation.y = q_cam_lidar.y();
    odom_trans.transform.rotation.z = q_cam_lidar.z();
    odom_trans.transform.rotation.w = q_cam_lidar.w();
    odometery_tf_publisher.sendTransform(odom_trans);

    // sensor/velodyne
    // sensor/camera
}