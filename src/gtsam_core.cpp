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

RobotxCalibration::RobotxCalibration() : body_ptx(0),
                                         body_pty(0),
                                         body_ptz(0),
                                         body_prx(0),
                                         body_pry(0),
                                         body_prz(0),
                                         accelerometer_sigma(0.01),
                                         gyroscope_sigma(0.000175),
                                         integration_sigma(0),
                                         accelerometer_bias_sigma(0.000167),
                                         gyroscope_bias_sigma(2.91e-006),
                                         average_delta_t(0.0100395199348279) 
                                         {}

RobotxCalibration::RobotxCalibration(const Vector12& calibration) : body_ptx(calibration(0)),
                                                                    body_pty(calibration(1)),
                                                                    body_ptz(calibration(2)),
                                                                    body_prx(calibration(3)),
                                                                    body_pry(calibration(4)),
                                                                    body_prz(calibration(5)),
                                                                    accelerometer_sigma(calibration(6)),
                                                                    gyroscope_sigma(calibration(7)),
                                                                    integration_sigma(calibration(8)),
                                                                    accelerometer_bias_sigma(calibration(9)),
                                                                    gyroscope_bias_sigma(calibration(10)),
                                                                    average_delta_t(calibration(11)) 
                                                                    {}

void GTSAM_CORE::initialize(Vector12& calibration, Vector3& position) {
	// init_state:: quaternion(qx,qy,qz,qw), anglular velocity(omegax, omegay, omegaz), linear acceleration(accx, accy, accz)

	robotx_calibration = RobotxCalibration(calibration);

    Vector6 BodyP = (Vector6() << robotx_calibration.body_ptx, robotx_calibration.body_pty, robotx_calibration.body_ptz,
                                  robotx_calibration.body_prx, robotx_calibration.body_pry, robotx_calibration.body_prz)
                    .finished();
    auto body_T_imu = Pose3::Expmap(BodyP);
    if (!body_T_imu.equals(Pose3(), 1e-5)) {
        printf("Currently only support IMUinBody is identity, i.e. IMU and body frame are the same");
        exit(-1);
    }

    // Configure different variables
    t1 = 0;
    GPS_update_count = 1;
    GPS_update_count_store = 1;
  	estimationPose_count = 1;
    gps_skip = 10;  // Skip this many GPS measurements each time
    g_ = 9.81; // 0 for reorded rosbag data, 9.81 for vrx simulation
    auto w_coriolis = Vector3::Zero();  // zero vector
    orientation = Vector4::Zero();

    // Configure noise models
    noise_model_gps = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                     Vector3::Constant(1.0/0.07))
                                                            .finished());

    // !!!TODO!!!
    // Set initial conditions for the estimated trajectory
    // initial pose is the reference frame (navigation frame)
    current_pose_global = Pose3(Rot3(), position);
    // the vehicle is stationary at the beginning at position 0,0,0
    current_velocity_global = Vector3::Zero();
    current_bias = imuBias::ConstantBias();  // init with zero bias

    auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                       Vector3::Constant(1.0e-6))   // fix prior pose
                                                         .finished());
    auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000.0));
    auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100),
                                                                   Vector3::Constant(5.00e-05))
                                                     .finished());
    //current_pose_cov = sigma_init_x;

    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov = I_3x3 * pow(robotx_calibration.accelerometer_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(robotx_calibration.gyroscope_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov = I_3x3 * pow(robotx_calibration.integration_sigma, 2);

    imu_params = boost::shared_ptr<PreintegrationParams>(PreintegrationParams::MakeSharedU(g_));
    //imu_params = std::make_shared<PreintegrationParams>(Vector3(0, 0, -g_));
    imu_params->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    imu_params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance = measured_omega_cov;       // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    current_summarized_measurement = boost::shared_ptr<PreintegratedImuMeasurements>(
                                        new PreintegratedImuMeasurements(imu_params, current_bias));
  
    //ISAM2Params isam_params;
    ISAM2Params isam_params;
    isam_params.factorization = ISAM2Params::QR;    //ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    isam2 = ISAM2(isam_params);
/*  
    auto current_pose_key = X(GPS_update_count);
    auto current_vel_key = V(GPS_update_count);
    auto current_bias_key = B(GPS_update_count);

    new_values.insert(current_pose_key, current_pose_global);
    new_values.insert(current_vel_key, current_velocity_global);
    new_values.insert(current_bias_key, current_bias);
    new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, current_pose_global, sigma_init_x);
    new_factors.emplace_shared<PriorFactor<Vector3>>(current_vel_key, current_velocity_global, sigma_init_v);
    new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key, current_bias, sigma_init_b);
*/
}

void GTSAM_CORE::addGPS(shared_ptr<PoseMeasurement> ptr) {

    if (fabs(orientation[0]) < 1e-5 && fabs(orientation[1]) < 1e-5 && fabs(orientation[2]) < 1e-5 && fabs(orientation[3]) < 1e-5) {
        printf("################ NO IMU UPDATE ################\n");
        printf("\n\n");
        return;
    }   

    double t = ptr -> getTime();
    Vector3 gps = ptr -> getData();

    auto current_pose_key = X(GPS_update_count);
    auto current_vel_key = V(GPS_update_count);
    auto current_bias_key = B(GPS_update_count);
    auto previous_pose_key = X(GPS_update_count - 1);
    auto previous_vel_key = V(GPS_update_count - 1);
    auto previous_bias_key = B(GPS_update_count - 1);

    //++GPS_update_count;

    if (GPS_update_count <= 1) {
        auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                       Vector3::Constant(1.0e-6))   // fix prior pose
                                                         .finished());
        auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000.0));
        auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100),
                                                                    Vector3::Constant(5.00e-05))
                                                        .finished());

        current_pose_global = Pose3(Rot3(Quaternion(orientation[3], orientation[0], orientation[1], orientation[2])), gps);

        new_values.insert(current_pose_key, current_pose_global);
        new_values.insert(current_vel_key, current_velocity_global);
        new_values.insert(current_bias_key, current_bias);

        new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, current_pose_global, sigma_init_x);
        new_factors.emplace_shared<PriorFactor<Vector3>>(current_vel_key, current_velocity_global, sigma_init_v);
        new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key, current_bias, sigma_init_b);
    
        current_summarized_measurement = boost::shared_ptr<PreintegratedImuMeasurements>(
                                        new PreintegratedImuMeasurements(imu_params, current_bias));
        included_imu_measurement_count = 0;

        ++GPS_update_count;
        return;
    }

    // Create IMU factor
    new_factors.emplace_shared<ImuFactor>(previous_pose_key, previous_vel_key,
                                          current_pose_key, current_vel_key,
                                          previous_bias_key, *current_summarized_measurement);
    //current_summarized_measurement -> print();

    //current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params, current_bias);
    current_summarized_measurement = boost::shared_ptr<PreintegratedImuMeasurements>(
                                        new PreintegratedImuMeasurements(imu_params, current_bias));
    
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
    //if ((GPS_update_count % gps_skip) == 0) {
    	new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, gps_pose, noise_model_gps);
        new_values.insert(current_pose_key, gps_pose);

        //printf("################ POSE INCLUDED AT TIME %lf ################\n", t);
        //gps_pose.translation().print();
        //printf("\n\n");
    //} else {
    //    new_values.insert(current_pose_key, current_pose_global);
    //}

    // Add initial values for velocity and bias based on the previous estimates
    new_values.insert(current_vel_key, current_velocity_global);
    new_values.insert(current_bias_key, current_bias);

    if (GPS_update_count > 1 + 2 * gps_skip){
        new_factors.print();
    	isam2.update(new_factors, new_values);
        printf("################ UPDATED ################\n");
        printf("\n\n");

        //Marginals marginals(new_factors, new_values);
        //current_pose_cov = marginals.marginalCovariance(current_pose_key);
        //printf("################ Marginals ################\n");
        //printf("\n\n");

        // Reset the newFactors and newValues list
        new_factors.resize(0);
        new_values.clear();

        printf("################ CLEARED ################\n");
        printf("\n\n");
    	result = isam2.calculateEstimate(); 

        printf("################ CALCULATED Estimate ################\n");
        printf("\n\n");

		current_pose_global = result.at<Pose3>(current_pose_key);
		current_velocity_global = result.at<Vector3>(current_vel_key);
		current_bias = result.at<imuBias::ConstantBias>(current_bias_key);
    }

    ++GPS_update_count;

}

void GTSAM_CORE::addIMU(shared_ptr<ImuMeasurement> ptr) {
    if (t1 == 0) {
        t1 = ptr -> getTime();
        return;
    }
  	double t2 = ptr -> getTime();
  	double dt = t2 - t1;
    t1 = t2;
  	Vector3 gyroscope = ptr -> getData().head(3);
  	Vector3 accelerometer = ptr -> getData().tail(3);
  	orientation = ptr -> getOri();

    twist_.head(3) = gyroscope * dt;
    twist_.tail(3) = accelerometer * dt;

    // twist_(0) = gyroscope(0)*dt;
    // twist_(1) = gyroscope(1)*dt;
    // twist_(2) = gyroscope(2)*dt;
    // twist_(3) = accelerometer(0)*dt;
    // twist_(4) = accelerometer(1)*dt;
    // twist_(5) = accelerometer(2)*dt;

	current_summarized_measurement -> integrateMeasurement(accelerometer, gyroscope, dt);
    ++included_imu_measurement_count;
	
}

void GTSAM_CORE::addLandmark(shared_ptr<LandmarkMeasurement> ptr) {
    auto current_pose_key_store = X(GPS_update_count_store);
    auto estimate_pose_key = E(estimationPose_count);
  	++estimationPose_count;

    int landmark_id = ptr -> getID();
    Vector3 landmark_gps_original = ptr -> getData(); //x y z
    Pose3 estimate_pose = ptr -> getPose();
  
    // 3 DoF, bearing should be normalized, Unit3 bearing
    auto bearingRangeNoise = noiseModel::Diagonal::Sigmas((Vector(3)<<0.01,0.03,0.05).finished());
    const SharedDiagonal noiseOdometery = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());
  
    Point3 landmark_gps = Point3(landmark_gps_original[0], landmark_gps_original[1], landmark_gps_original[2]);
  	auto bearing11 = estimate_pose.bearing(landmark_gps);
    auto range11 = estimate_pose.range(landmark_gps);
  
    if (landmark_id_to_key.find(landmark_id) == landmark_id_to_key.end()) {
        // Add initial (prior) landmark at first detection
      	landmark_id_to_key[landmark_id] = landmark_count;
      	++landmark_count;
      
        new_values.insert(L(landmark_count), landmark_gps);
    }
    else{
      // estimation pose - landmark
      new_factors.emplace_shared<BearingRangeFactor<Pose3, Point3> >(
            estimate_pose_key, L(landmark_id_to_key[landmark_id]), bearing11, range11, bearingRangeNoise);
      
      // estimation pose - current global pose
      new_factors.push_back(BetweenFactor<Pose3>(estimate_pose_key, current_pose_key_store, estimate_pose.between(current_pose_global), noiseOdometery));
      
      printf("################ Added Estimation Pose ################\n");
      current_pose_global.print();
	  estimate_pose.print();
      estimate_pose.between(current_pose_global).print();

    }

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

Vector6 GTSAM_CORE::getTwist() {
  	return twist_;
}  

}   // end of namespace GTSAM
