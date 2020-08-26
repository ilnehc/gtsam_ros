/* ----------------------------------------------------------------------------
 * Copyright 2020, Li Chen <ilnehc@umich.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gtsam_core.cpp
 *  @author Li Chen
 *  @brief  Source file for the core of the GTSAM
 *  @date   August 19, 2020
 **/

#include "gtsam_core.h"
using namespace gtsam;
using namespace std;

namespace GTSAM {

void GTSAM_CORE::initialize(Vector12& calibration, Vector3& position) {
	// init_state:: quaternion(qx,qy,qz,qw), anglular velocity(omegax, omegay, omegaz), linear acceleration(accx, accy, accz)

	robotx_calibration = RobotxCalibration(calibration);

    Vector6 BodyP = (Vector6() << calibration.body_ptx, calibration.body_pty, calibration.body_ptz,
                                  calibration.body_prx, calibration.body_pry, calibration.body_prz)
                    .finished();
    auto body_T_imu = Pose3::Expmap(BodyP);
    if (!body_T_imu.equals(Pose3(), 1e-5)) {
        printf("Currently only support IMUinBody is identity, i.e. IMU and body frame are the same");
        exit(-1);
    }

    // Configure different variables
    GPS_update_count = 1;
    gps_skip = 10;  // Skip this many GPS measurements each time
    g_ = 9.8;
    auto w_coriolis = Vector3::Zero();  // zero vector

    // Configure noise models
    noise_model_gps = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                     Vector3::Constant(1.0/0.07))
                                                            .finished());

    // Set initial conditions for the estimated trajectory
    // initial pose is the reference frame (navigation frame)
    current_pose_global = Pose3(Rot3(), position);
    // the vehicle is stationary at the beginning at position 0,0,0
    current_velocity_global = Vector3::Zero();
    current_bias = imuBias::ConstantBias();  // init with zero bias

    auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                       Vector3::Constant(1.0))
                                                         .finished());
    auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000.0));
    auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100),
                                                                   Vector3::Constant(5.00e-05))
                                                     .finished());
    current_pose_cov = sigma_init_x;

    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov = I_3x3 * pow(calibration.accelerometer_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(calibration.gyroscope_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov = I_3x3 * pow(calibration.integration_sigma, 2);

    imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g_);
    imu_params->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    imu_params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance = measured_omega_cov;       // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    current_summarized_measurement = nullptr;
  
    //ISAM2Params isam_params;
    isam_params.factorization = ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    isam2 = ISAM2(isam_params);
  
    auto current_pose_key = X(GPS_update_count);
    auto current_vel_key = V(GPS_update_count);
    auto current_bias_key = B(GPS_update_count);
  
    new_values.insert(current_pose_key, current_pose_global);
    new_values.insert(current_vel_key, current_velocity_global);
    new_values.insert(current_bias_key, current_bias);
    new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, current_pose_global, sigma_init_x);
    new_factors.emplace_shared<PriorFactor<Vector3>>(current_vel_key, current_velocity_global, sigma_init_v);
    new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key, current_bias, sigma_init_b);

}

void GTSAM_CORE::addGPS(shared_ptr<PoseMeasurement> ptr) {
    ++GPS_update_count;
    double t = ptr -> getTime();
    Vector3 gps = ptr -> getData();

    auto current_pose_key = X(GPS_update_count);
    auto current_vel_key = V(GPS_update_count);
    auto current_bias_key = B(GPS_update_count);
    auto previous_pose_key = X(GPS_update_count - 1);
    auto previous_vel_key = V(GPS_update_count - 1);
    auto previous_bias_key = B(GPS_update_count - 1);
    
    current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params, current_bias);
    
    // Create IMU factor
    new_factors.emplace_shared<ImuFactor>(previous_pose_key, previous_vel_key,
                                          current_pose_key, current_vel_key,
                                          previous_bias_key, *current_summarized_measurement);
    
    // Bias evolution as given in the IMU metadata
    auto sigma_between_b = noiseModel::Diagonal::Sigmas((Vector6() <<
            Vector3::Constant(sqrt(included_imu_measurement_count) * robotx_calibration.accelerometer_bias_sigma),
            Vector3::Constant(sqrt(included_imu_measurement_count) * robotx_calibration.gyroscope_bias_sigma))
            .finished());
    new_factors.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(previous_bias_key,
                                                                     current_bias_key,
                                                                     imuBias::ConstantBias(),
                                                                     sigma_between_b);
    included_imu_measurement_count = 0; // set the counter to 0

    // Create GPS factor
    auto gps_pose = Pose3(current_pose_global.rotation(), gps);
    if ((GPS_update_count % gps_skip) == 0) {
    	new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, gps_pose, noise_model_gps);
        new_values.insert(current_pose_key, gps_pose);

        //printf("################ POSE INCLUDED AT TIME %lf ################\n", t);
        //gps_pose.translation().print();
        //printf("\n\n");
    } else {
        new_values.insert(current_pose_key, current_pose_global);
    }

    // Add initial values for velocity and bias based on the previous estimates
    new_values.insert(current_vel_key, current_velocity_global);
    new_values.insert(current_bias_key, current_bias);

    if (GPS_update_count > 1 + 2 * gps_skip){
    	isam2.update(new_factors, new_values);
        Marginals marginals(new_factors, new_values);
        current_pose_cov = marginals.marginalCovariance(current_pose_key);
        // Reset the newFactors and newValues list
        new_factors.resize(0);
        new_values.clear();
    	result = isam2.calculateEstimate(); 
		current_pose_global = result.at<Pose3>(current_pose_key);
		current_velocity_global = result.at<Vector3>(current_vel_key);
		current_bias = result.at<imuBias::ConstantBias>(current_bias_key);
    }

}

void GTSAM_CORE::addIMU(shared_ptr<ImuMeasurement> ptr) {
	t1 = t2;
  	t2 = ptr -> getTime();
  	dt = t2 - t1;
  	Vector3 gyroscope = ptr -> getData().head(3);
  	Vector3 accelerometer = ptr -> getData().tail(3);
  	Vector4 orientation = ptr -> getOri();
  
	current_summarized_measurement -> integrateMeasurement(accelerometer, gyroscope, dt);
    ++included_imu_measurement_count;
	
}

Values GTSAM_CORE::getResult() {
  	return result;
}  

Pose3 GTSAM_CORE::getCurPose() {  
	return current_pose_global; 
}

Matrix GTSAM_CORE::getMarginalPoseCov() {
    return current_pose_cov;
}



}   // end of namespace GTSAM