#ifndef POSE_GT_H
#define POSE_GT_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

// Declare the global publisher
extern ros::Publisher pub_posegt;

// Function to register the publisher
void registerPub_gt(ros::NodeHandle &n);

// Function to publish the pose ground truth
void pubPosegt(const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v, Eigen::Matrix<double, 15, 15> &covariance, double t);

#endif // POSE_GT_H
