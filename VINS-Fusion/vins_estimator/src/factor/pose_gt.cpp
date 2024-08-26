#include "../factor/pose_gt.h"
using namespace ros;
ros::Publisher pub_posegt;

void registerPub_gt(ros::NodeHandle &n)
{
    pub_posegt = n.advertise<nav_msgs::Odometry>("pose_gt", 50);
}

void pubPosegt(const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v, Eigen::Matrix<double, 15, 15> &covariance)
{
    nav_msgs::Odometry odom_msg;
    ros::Time current_time = ros::Time::now();
    // Fill header
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Position
    odom_msg.pose.pose.position.x = delta_p.x();
    odom_msg.pose.pose.position.y = delta_p.y();
    odom_msg.pose.pose.position.z = delta_p.z();

    // Orientation
    Eigen::Quaterniond orientation = delta_q;
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();

    // Velocity
    odom_msg.twist.twist.linear.x = delta_v.x();
    odom_msg.twist.twist.linear.y = delta_v.y();
    odom_msg.twist.twist.linear.z = delta_v.z();

    // Angular velocity (not directly available in your current IntegrationBase class, 
    // but you might want to add this in a similar fashion to linear velocities if needed)
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    // Covariance matrices
    //Eigen::Matrix<double, 15, 15> covariance = covariance;
    
    // Pose covariance (position + orientation)
    // Position (x, y, z) - Mapping directly from the first 3x3 block of covariance
    odom_msg.pose.covariance[0] = covariance(0, 0); // x position variance
    odom_msg.pose.covariance[1] = covariance(0, 1); // x position / y position covariance
    odom_msg.pose.covariance[2] = covariance(0, 2); // x position / z position covariance
    odom_msg.pose.covariance[7] = covariance(1, 1); // y position variance
    odom_msg.pose.covariance[8] = covariance(1, 2); // y position / z position covariance
    odom_msg.pose.covariance[14] = covariance(2, 2); // z position variance
    
    // Orientation (roll, pitch, yaw) - Simplified as a diagonal model
    // Assuming that these values are based on a simplified model of orientation
    // Adjust indices to reflect quaternion covariance correctly
    odom_msg.pose.covariance[35] = covariance(9, 9); // roll variance
    odom_msg.pose.covariance[31] = covariance(10, 10); // pitch variance
    odom_msg.pose.covariance[30] = covariance(11, 11); // yaw variance
    
    // Twist covariance (linear + angular velocities)
    // Linear velocities (x, y, z) - Mapping directly from the 3x3 block of covariance
    odom_msg.twist.covariance[0] = covariance(6, 6); // x velocity variance
    odom_msg.twist.covariance[1] = covariance(6, 7); // x velocity / y velocity covariance
    odom_msg.twist.covariance[2] = covariance(6, 8); // x velocity / z velocity covariance
    odom_msg.twist.covariance[7] = covariance(7, 7); // y velocity variance
    odom_msg.twist.covariance[8] = covariance(7, 8); // y velocity / z velocity covariance
    odom_msg.twist.covariance[14] = covariance(8, 8); // z velocity variance```
    // Publish
    pub_posegt.publish(odom_msg);
}