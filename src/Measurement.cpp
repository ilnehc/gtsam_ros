/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Measurement class
 *  @date   September 27, 2018
 **/

#include "Measurement.h"
using namespace std;

// Construct Empty Measurement
Measurement::Measurement() {
    t_ = 0;
    type_ = EMPTY;
}
// Getters
double Measurement::getTime() { return t_; }
MeasurementType Measurement::getType() { return type_; }

// Construct IMU measurement
ImuMeasurement::ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg) {
    t_ = msg->header.stamp.toSec();
    data_ << msg->angular_velocity.x, 
             msg->angular_velocity.y, 
             msg->angular_velocity.z,
             msg->linear_acceleration.x, 
             msg->linear_acceleration.y, 
             msg->linear_acceleration.z;
    ori_ << msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w;
    type_ = IMU;
}
Eigen::VectorXd ImuMeasurement::getData() { return data_; }
Eigen::VectorXd ImuMeasurement::getOri() { return ori_; }

// Construct GPS measurement
GpsMeasurement::GpsMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    t_ = msg->header.stamp.toSec();
    data_ << msg->latitude, 
             msg->longitude, 
             msg->altitude;
    type_ = GPS;
}
Eigen::VectorXd GpsMeasurement::getData() { return data_; }

// Construct Pose measurement
PoseMeasurement::PoseMeasurement(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    t_ = msg->header.stamp.toSec();
    data_ << msg->pose.pose.position.x, 
             msg->pose.pose.position.y, 
             msg->pose.pose.position.z;
    ori_ << msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w;
    type_ = POSE;
}
Eigen::VectorXd PoseMeasurement::getData() { return data_; }

// Construct ground truth LinkStates measurement
GtLinkMeasurement::GtLinkMeasurement(const gazebo_msgs::LinkStates::ConstPtr& msg, const double t) {
    t_ = t;
    int n = (msg->pose).size();
    pos_ << msg->pose[n-9].position.x,  // 8 is gps_link, 9 is base_link
            msg->pose[n-9].position.y, 
            msg->pose[n-9].position.z;
    ori_ << msg->pose[n-9].orientation.x,
            msg->pose[n-9].orientation.y,
            msg->pose[n-9].orientation.z,
            msg->pose[n-9].orientation.w;
    type_ = LINK;
}
Eigen::VectorXd GtLinkMeasurement::getPos() { return pos_; }
Eigen::VectorXd GtLinkMeasurement::getOri() { return ori_; }

// Construct Landmark measurement
LandmarkMeasurement::LandmarkMeasurement(const int id, const Eigen::Vector3d& data, 
        const gtsam::Pose3& pose, const double t) : id_(id), data_(data), pose_(pose) {
    t_ = t;
    type_ = LANDMARK;
}
int LandmarkMeasurement::getID() { return id_; }
Eigen::VectorXd LandmarkMeasurement::getData() { return data_; }
gtsam::Pose3 LandmarkMeasurement::getPose() { return pose_; }

// Print measurement
ostream& operator<<(ostream& os, const Measurement& m) {
    string type_str;
    switch (m.type_) {
        case IMU :
            type_str = "IMU";
            break;
        case GPS :
            type_str = "GPS";
            break;
        case LANDMARK :
            type_str = "LANDMARK";
            break;
        default:
            type_str = "Unknown";
    }
    os << "Measurement type: " << type_str << endl;
    return os;
}
