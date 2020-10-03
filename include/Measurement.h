/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.h
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf/transform_listener.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

enum MeasurementType {EMPTY, IMU, GPS, POSE, LINK, LANDMARK};


class Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        double getTime();
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        double t_;
        MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(const std::shared_ptr<Measurement> lhs, const std::shared_ptr<Measurement> rhs) const {
    return lhs->getTime() > rhs->getTime();
  }
};


class ImuMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
        Eigen::VectorXd getData();
        Eigen::VectorXd getOri();

    private: 
        Eigen::Matrix<double,6,1> data_;    // w a
        Eigen::Matrix<double,4,1> ori_; // x y z w
};

class GpsMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GpsMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);
        Eigen::VectorXd getData();
        Eigen::VectorXd getOri();

    private: 
        Eigen::Matrix<double,3,1> data_;
};

class PoseMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PoseMeasurement(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        Eigen::VectorXd getData();

    private: 
        Eigen::Matrix<double,3,1> data_;    // x y z
        Eigen::Matrix<double,4,1> ori_;     // x y z w
};

class GtLinkMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GtLinkMeasurement(const gazebo_msgs::LinkStates::ConstPtr& msg, const double t);
        Eigen::VectorXd getPos();
        Eigen::VectorXd getOri();

    private: 
        Eigen::Matrix<double,3,1> pos_; // x y z
        Eigen::Matrix<double,4,1> ori_; // x y z w
};


class LandmarkMeasurement : public Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LandmarkMeasurement(const int id, const Eigen::Vector3d& data, const gtsam::Pose3& pose, const double t);
        int getID();
        Eigen::VectorXd getData();
        gtsam::Pose3 getPose();

    private:
        int id_;
        Eigen::Matrix<double,3,1> data_;  // x y z
        gtsam::Pose3 pose_;     // corresponding robot pose
};


#endif 
