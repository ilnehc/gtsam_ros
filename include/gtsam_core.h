/* ----------------------------------------------------------------------------
 * Copyright 2020, Li Chen <ilnehc@umich.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gtsam_core.h
 *  @author Li Chen
 *  @brief  Header file for the core of the GTSAM
 *  @date   August 19, 2020
 **/

#ifndef GTSAM_CORE_H
#define GTSAM_CORE_H
#include <memory>
#include <chrono>
#include <string>
#include <boost/lockfree/queue.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#if GTSAM_USE_MUTEX
#include <mutex>
#endif
#include <algorithm>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/inference/Symbol.h>

#include "Measurement.h"

using namespace gtsam;
using namespace std;
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::L;  // Landmark Point3 (x,y,z)
using symbol_shorthand::E;	// Inserted estimation Pose3 (x,y,z,r,p,y)

namespace GTSAM {

class RobotxCalibration {
public:
    double body_ptx;
    double body_pty;
    double body_ptz;
    double body_prx;
    double body_pry;
    double body_prz;
    double accelerometer_sigma;
    double gyroscope_sigma;
    double integration_sigma;
    double accelerometer_bias_sigma;
    double gyroscope_bias_sigma;
    double average_delta_t;

    RobotxCalibration();
  	RobotxCalibration(const Vector12& calibration);
};

class GTSAM_CORE {
public:
    void initialize(Vector12& calibration, Vector3& position);
    Pose3 getCurPose();
  	Values getResult();
    Matrix getMarginalPoseCov();
    Vector6 getTwist();

    // x, y, z
  	void addGPS(shared_ptr<PoseMeasurement> ptr);	
  	// quaternion(qx,qy,qz,qw), anglular velocity(omegax, omegay, omegaz), linear acceleration(accx, accy, accz)
    void addIMU(shared_ptr<ImuMeasurement> ptr);
    void addLandmark(shared_ptr<LandmarkMeasurement> ptr);

private:
	  double t1;
    size_t included_imu_measurement_count;
  	double g_;
    uint64_t GPS_update_count;
    uint64_t GPS_update_count_store;
  	int gps_skip;
    uint64_t landmark_count;
    uint64_t estimationPose_count;
    Vector6 twist_;

    unordered_map<int, int> landmark_id_to_key;	// (landmark_id, corresponding landmark_count);

    boost::shared_ptr<PreintegrationParams> imu_params;
    boost::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement;
    noiseModel::Diagonal::shared_ptr noise_model_gps;
	  imuBias::ConstantBias current_bias;
    Pose3 current_pose_global;
    Matrix current_pose_cov;
    Vector3 current_velocity_global;
    RobotxCalibration robotx_calibration;
    Vector4 orientation;

    ISAM2 isam2;
	  Values result;  
    // Create the factor graph and values object that will store new factors and values to add to the incremental graph
    NonlinearFactorGraph new_factors;
    Values new_values;
};

}   // end of namespace GTSAM

#endif 
