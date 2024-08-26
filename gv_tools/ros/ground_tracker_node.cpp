#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "ground_tracker.h"
#include "simple_compensation.hpp"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

using namespace std;

queue<pair<double, cv::Mat>> img_queue;
queue<pair<double, Eigen::Matrix4d>> pose_queue;

ros::Subscriber sub_img;
ros::Subscriber sub_pose;
image_transport::Publisher pub_img_track;
image_transport::Publisher pub_ipm_track;
ros::Publisher features_pub;
shared_ptr<gv::GroundTracker> tracker;

std::map<double, double> all_imu_pitch;
bool enable_pitch_comp;
double pitch_comp_windowsize;
Eigen::Matrix4d last_pose;
//Conversion function
void convertMapToPointCloud(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& input_map, sensor_msgs::PointCloudPtr& point_cloud_msg)
{

    point_cloud_msg->points.clear();
    for (const auto& entry : input_map)
    {
        int feature_id = entry.first;
        for (const auto& pair : entry.second)
        {

            int camera_id = pair.first;
            const Eigen::Matrix<double, 7, 1>& data = pair.second;

            point_cloud_msg->channels[0].values[feature_id] = feature_id;
            point_cloud_msg->channels[1].values[feature_id] = camera_id;
            point_cloud_msg->channels[2].values[feature_id] = data(3);
            point_cloud_msg->channels[3].values[feature_id] = data(4);
            point_cloud_msg->channels[4].values[feature_id] = data(5);
            point_cloud_msg->channels[5].values[feature_id] = data(6);
            // Assuming the first three elements of the matrix are x, y, z coordinates
            point_cloud_msg->points[feature_id].x = data(0);
            point_cloud_msg->points[feature_id].y = data(1);
            point_cloud_msg->points[feature_id].z = data(2);

        }
    }
}


void img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat mm = cv_bridge::toCvCopy(msg, "bgr8")->image;
    img_queue.push(make_pair(msg->header.stamp.toSec(), mm));

    // Note: For VIO implementation, process IMU data here.
    // Feed the pose prediction into feature tracker.

    ros::Rate wait_for_pose(100);
    while (pose_queue.empty() && ros::ok())
    {
        ROS_WARN("Waiting for pose!!!");
        wait_for_pose.sleep();
    }

    // **Attention**: Assuming synced image and pose inputs here.
    while (!pose_queue.empty() && !img_queue.empty()&& ros::ok())
    {
        if (pose_queue.front().first <= img_queue.front().first - 1e-2)
            pose_queue.pop();
        else if (pose_queue.front().first >= img_queue.front().first + 1e-2)
            img_queue.pop();
        else
        {   
            ROS_INFO("Get synced data!!! %.5lf %.5lf",pose_queue.front().first, img_queue.front().first);
            double tt = pose_queue.front().first;
            cv::Mat img = img_queue.front().second;
            Eigen::Matrix4d Twik = pose_queue.front().second;
            Eigen::Matrix4d Twik_1 = last_pose;
            Eigen::Matrix4d Tckck_1 = (Twik * tracker->config.Tic).inverse() * (Twik_1 * tracker->config.Tic);

            pose_queue.pop();
            img_queue.pop();

            
            gv::CameraGroundGeometry cg = tracker->config.cg;
            // Calculate temporary camera-ground geometry considering
            // high-frequency pitch compenstation
            if (enable_pitch_comp)
            {
                all_imu_pitch[tt] = gv::m2att(Twik.block<3, 3>(0, 0)).x() * 180 / M_PI;
                double comp = simple_pitch_compenstation(tt, all_imu_pitch, pitch_comp_windowsize);
                cg.update(cg.getAlpha(), cg.getTheta() + comp);
            }
            
            // main feature tracking
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
            std::pair<map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>,double> trackImageOutput = tracker->trackImage(tt, img, 10.0, cg, Tckck_1, false);
            std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>  feature_frame = trackImageOutput.first;
            // Create a PointCloud message
            sensor_msgs::PointCloudPtr point_cloud_msg(new sensor_msgs::PointCloud);

            // Set the frame ID and timestamp
            point_cloud_msg->header.frame_id = "world"; // Replace with your frame
            ros::Time ros_time = ros::Time(trackImageOutput.second);
            point_cloud_msg->header.stamp = ros_time; //timestamp of the tracked image

            // Example input data - replace with your actual map
            std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> input_map;
            input_map = feature_frame;

            // Convert the map to PointCloud message
            //convertMapToPointCloud(input_map, point_cloud_msg);
            // point_cloud_msg->points.clear();
            int size = feature_frame.size();
            
            std::vector<float> values1(size), values2(size), values3(size), values4(size), values5(size), values6(size);
            sensor_msgs::ChannelFloat32 channel1, channel2, channel3, channel4, channel5, channel6;

            for (auto& entry : feature_frame) 
            {
                geometry_msgs::Point32 point;
                int feature_id = entry.first; //feature.id0
                std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>& vec = entry.second; //[()]
                std::cout << "Feature ID: " << feature_id << std::endl;
                for (const auto& pair : vec) // ()
                {

                    int camera_id = pair.first; // (0,)
                    const Eigen::Matrix<double, 7, 1>& data = pair.second;  // (0 , 7*1)
                    point.x = data(0); //x
                    point.y = data(1); //y
                    point.z = data(2); //z
                    values2.push_back(camera_id);
                    values3.push_back(data(3)); //u
                    values4.push_back(data(4)); //v
                    values5.push_back(data(5)); //vx
                    values6.push_back(data(6)); //vy
                    
                    point_cloud_msg->points.push_back(point);

                }
                
                values1.push_back(feature_id);
                
            }
            channel1.name = "feature_id";
            channel2.name = "camera_id";
            channel3.name = "p_u";
            channel4.name = "p_v";
            channel5.name = "velocity_x";
            channel6.name = "velocity_y";
            // Assign the float vectors to the channels
            channel1.values = values1;
            channel2.values = values2;
            channel3.values = values3;
            channel4.values = values4;
            channel5.values = values5;
            channel6.values = values6;
            point_cloud_msg->channels.push_back(channel1);
            point_cloud_msg->channels.push_back(channel2);
            point_cloud_msg->channels.push_back(channel3);
            point_cloud_msg->channels.push_back(channel4);
            point_cloud_msg->channels.push_back(channel5);
            point_cloud_msg->channels.push_back(channel6);
            // // Publish the message
            features_pub.publish(point_cloud_msg);

            // visualization
            std_msgs::Header header;
            header.stamp = ros::Time(tt);
            header.frame_id = "cam0";
            sensor_msgs::ImagePtr img_show_msg = cv_bridge::CvImage(header, "bgr8", tracker->cur_img_show).toImageMsg();
            sensor_msgs::ImagePtr ipm_show_msg = cv_bridge::CvImage(header, "bgr8", tracker->cur_ipm_show).toImageMsg();
            pub_img_track.publish(img_show_msg);
            pub_ipm_track.publish(ipm_show_msg);

            last_pose = Twik;
            
        }
    }
}

void imu_pose_callback(const nav_msgs::Odometry &msg)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = Eigen::Quaterniond(
                                 msg.pose.pose.orientation.w,
                                 msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z)
                                 .toRotationMatrix();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(msg.pose.pose.position.x,
                                             msg.pose.pose.position.y,
                                             msg.pose.pose.position.z);
    pose_queue.push(make_pair(msg.header.stamp.toSec(), pose));
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_tracker_node");
    ros::NodeHandle nh("~");
    
    string config_path;
    nh.getParam("config_path", config_path);
    ROS_WARN("config_path: %s\n",config_path.c_str());
    cv::FileStorage fsSettings(config_path, cv::FileStorage::READ);
    enable_pitch_comp = (int)fsSettings["enable_pitch_comp"];
    pitch_comp_windowsize = fsSettings["pitch_comp_windowsize"];
    fsSettings.release();

    gv::CameraConfig config(config_path);
    tracker = make_shared<gv::GroundTracker>(config);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    sub_img = nh.subscribe("/front/realsense/color/image_raw", 1, img_callback);
    sub_pose = nh.subscribe("/odom", 1, imu_pose_callback);
    image_transport::ImageTransport it(nh);
    // Create a publisher for the PointCloud message
    features_pub = nh.advertise<sensor_msgs::PointCloud>("/ground_tracker_node/features", 2000);
    pub_img_track = it.advertise("img_track", 2);
    pub_ipm_track = it.advertise("ipm_track", 2);

    ros::waitForShutdown();
}

