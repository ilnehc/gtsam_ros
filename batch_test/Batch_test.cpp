/**
 * @file Batch_test
 * @brief Batch test for IMU and GPS based on IMUKittiExampleGPS
 * @author Chao Chen, Li Chen, Shoutian Wang
 */

// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <cstring>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)

struct ImuMeasurement {
    double time;
    double dt;
    Vector3 accelerometer;
    Vector3 gyroscope;  // omega
    Vector4 orientation;
};

struct GpsMeasurement {
    double time;
    Vector3 position;  // x,y,z
};

const string output_filename = "/home/chaochen/Desktop/NA568_PS4_Key/src/BatchTestResults.csv";

Vector4 imuapproximate(double time, vector<ImuMeasurement>& imu_measurements ){
    Vector4 result;
    //Quaternion qrep;
    double gradient;
    for(int i=0; i<imu_measurements.size();i++){
        /*
        if (time >imu_measurements[i].time){
            printf("qa1_time: %18.10lf\n", imu_measurements[i].time);
            printf("qb1_time: %18.10lf\n", imu_measurements[i+1].time);
            printf("qrep1_time: %18.10lf\n", time);
        }
        */
    	if ((time >imu_measurements[i].time)&&(time <=imu_measurements[i+1].time)){
	    
            for(int j=0; j<4; j++){
                gradient = (imu_measurements[i+1].orientation[j] - imu_measurements[i].orientation[j]) / (imu_measurements[i+1].time - imu_measurements[i].time);
                result[j] = imu_measurements[i].orientation[j] + gradient * (time - imu_measurements[i].time);
	        } 
            
            //cout << "checkchek2" <<endl;

            //Quaternion qa(imu_measurements[i].orientation[3], imu_measurements[i].orientation[0], imu_measurements[i].orientation[1], imu_measurements[i].orientation[2]), 
            //           qb(imu_measurements[i+1].orientation[3], imu_measurements[i+1].orientation[0], imu_measurements[i+1].orientation[1], imu_measurements[i+1].orientation[2]);
            //cout << "checkchek3" <<endl;
            //qrep = qa.slerp((time-imu_measurements[i].time)/(imu_measurements[i+1].time - imu_measurements[i].time), qb);
            //cout << "qa: " << imu_measurements[i].orientation << endl;
            //cout << "qb: " << imu_measurements[i+1].orientation << endl;
            //cout << "qrep: " << result << endl;
            //printf("qa2_time: %18.10lf\n", imu_measurements[i].time);
            //printf("qb2_time: %18.10lf\n", imu_measurements[i+1].time);
            //printf("qrep2_time: %18.10lf\n", time);
	        return result;
        }
    }
    return result;
}


void loadRobotxData(vector<ImuMeasurement>& imu_measurements,
                    vector<GpsMeasurement>& gps_measurements) {
  	string line;
  
    // Read IMU data
    // Time dt accelX accelY accelZ omegaX omegaY omegaZ
    string imu_data_file = "/home/chaochen/Desktop/NA568_PS4_Key/src/imu_92s_true.csv";
    printf("-- Reading IMU measurements from file\n");
    ifstream imu_data(imu_data_file);
  
    getline(imu_data, line, '\n');  // ignore the first line
    //cout << "[ " << line << " ]" << endl;

    double time = 0, dt = 0, ori_x = 0, ori_y = 0, ori_z = 0, ori_w = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0, acc_x = 0, acc_y = 0, acc_z = 0;
    double t = (double)1541972781600182016 / 1e9;
    while (getline(imu_data, line, '\n')) {
        //getline(imu_data, line, '\n');
        stringstream ss(line);
        string tmp;
        getline(ss, tmp, ',');
        time = stod(tmp);
        getline(ss, tmp, ',');
        ori_x = stod(tmp);
        getline(ss, tmp, ',');
        ori_y = stod(tmp);
        getline(ss, tmp, ',');
        ori_z = stod(tmp);
        getline(ss, tmp, ',');
        ori_w = stod(tmp);
        getline(ss, tmp, ',');
        gyro_x = stod(tmp);
        getline(ss, tmp, ',');
        gyro_y = stod(tmp);
        getline(ss, tmp, ',');
        gyro_z = stod(tmp);
        getline(ss, tmp, ',');
        acc_x = stod(tmp);
        getline(ss, tmp, ',');
        acc_y = stod(tmp);
        getline(ss, tmp, ',');
        acc_z = stod(tmp);
	/*
        sscanf(line.c_str(), "%lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf[^,] %lf",
               &time,
               &ori_x, &ori_y, &ori_z, &ori_w,
               &gyro_x, &gyro_y, &gyro_z,
               &acc_x, &acc_y, &acc_z);
	*/
        ImuMeasurement measurement;
        measurement.time = time / 1e9;
        measurement.dt = measurement.time - t;
      	t = measurement.time;
	    //printf("checkpoint1: %d \n", count);
        measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
        measurement.gyroscope = Vector3(gyro_x, gyro_y, gyro_z);
        measurement.orientation = Vector4(ori_x, ori_y, ori_z, ori_w);

        imu_measurements.push_back(measurement);

    }

    // Read GPS data
    // Time,X,Y,Z
        string gps_data_file = "/home/chaochen/Desktop/NA568_PS4_Key/src/gps_92s_xyz_true.csv";
        printf("-- Reading GPS measurements from file\n");
        ifstream gps_data(gps_data_file.c_str());
        getline(gps_data, line, '\n');  // ignore the first line
        double gps_x = 0, gps_y = 0, gps_z = 0;
        while (getline(gps_data, line, '\n')) {
            // based on data, the frequency can't be too close to IMU's
            getline(gps_data, line, '\n');
            getline(gps_data, line, '\n');
            getline(gps_data, line, '\n');
            if (line.empty()){
                break;
            }
            stringstream ss(line);
            string tmp;
            getline(ss, tmp, ',');
            time = stod(tmp);
            getline(ss, tmp, ',');
            gps_x = stod(tmp);
            getline(ss, tmp, ',');
            gps_y = stod(tmp);
            getline(ss, tmp, ',');
            gps_z = stod(tmp);
            //sscanf(line.c_str(), "%lf[^,] %lf[^,] %lf[^,] %lf", &time, &gps_x, &gps_y, &gps_z);

            GpsMeasurement measurement;
            measurement.time = time / 1e9;
            measurement.position = Vector3(gps_x, gps_y, gps_z);
            gps_measurements.push_back(measurement);
    }
}

int main(int argc, char* argv[]) {
    vector<ImuMeasurement> imu_measurements;
    vector<GpsMeasurement> gps_measurements;
    loadRobotxData(imu_measurements, gps_measurements);
    printf("imu_measurements: %ld\n",imu_measurements.size());
    printf("gps_measurements: %ld\n",gps_measurements.size());
  
    double body_ptx = 0;
    double body_pty = 0;
    double body_ptz = 0;
    double body_prx = 0;
    double body_pry = 0;
    double body_prz = 0; 
    double accelerometer_sigma = 0.1;//0.01;  
    double gyroscope_sigma = 0.000175;
    double integration_sigma = 0;
    double accelerometer_bias_sigma = 0.000167;
    double gyroscope_bias_sigma = 1e-5;//2.91e-006;
    double average_delta_t = 0.0100395199348279;
  
    //Initial pose
    Vector6 BodyP = (Vector6() << body_ptx, body_pty, body_ptz,
                                  body_prx, body_pry, body_prz)
                    .finished();
    auto body_T_imu = Pose3::Expmap(BodyP);
    if (!body_T_imu.equals(Pose3(), 1e-5)) {
        printf("Currently only support IMUinBody is identity, i.e. IMU and body frame are the same");
        exit(-1);
    }
    // Configure different variables
    // double t_offset = gps_measurements[0].time;
    size_t first_gps_pose = 1;
    size_t gps_skip = 10;  // Skip this many GPS measurements each time
    double g = 0;
    auto w_coriolis = Vector3::Zero();  // zero vector

    // Configure noise models
    auto noise_model_gps = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0),
                                                                          Vector3::Constant(1.0/0.07))
                                                            .finished());
    // Set initial conditions for the estimated trajectory
    // initial pose is the reference frame (navigation frame)

    //auto current_pose_global = Pose3(Rot3(), gps_measurements[0].position);
    Vector4 current_rotation = imuapproximate(gps_measurements[0].time, imu_measurements);
    auto current_pose_global = Pose3(Rot3(Quaternion(current_rotation[3], current_rotation[0], current_rotation[1], current_rotation[2])), gps_measurements[0].position);
    Vector4 current_rotation2 = imuapproximate(gps_measurements[1].time, imu_measurements);
    auto current_pose_global2 = Pose3(Rot3(Quaternion(current_rotation2[3], current_rotation2[0], current_rotation2[1], current_rotation2[2])), gps_measurements[1].position);

    // the vehicle is stationary at the beginning at position 0,0,0
    Vector3 current_velocity_global = Vector3::Zero();
    Vector3 current_velocity_global2 = Vector3::Zero();
    current_velocity_global2[0] = 0.1;
    current_velocity_global2[1] = 0.1;

    auto current_bias = imuBias::ConstantBias();  // init with zero bias

    auto sigma_init_x = noiseModel::Diagonal::Precisions((Vector6() << Vector3::Constant(0.1),	//0
                                                                       Vector3::Constant(1.0e-6))  //1.0
                                                         .finished());
    auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1.0));	//1000
    auto sigma_init_b = noiseModel::Diagonal::Sigmas((Vector6() << Vector3::Constant(0.100),
                                                                   Vector3::Constant(5.00e-5))	//5.00e-5
                                                     .finished());
    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov = I_3x3 * pow(accelerometer_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(gyroscope_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov = I_3x3 * pow(integration_sigma, 2);
    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
    imu_params->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    imu_params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
    imu_params->gyroscopeCovariance = measured_omega_cov;       // gyro white noise in continuous
    imu_params->omegaCoriolis = w_coriolis;

    std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement = nullptr;

    // Set ISAM2 parameters and create ISAM2 solver object
  
    //ISAM2Params isam_params;
    //isam_params.factorization = ISAM2Params::CHOLESKY;
    //isam_params.relinearizeSkip = 10;

    //ISAM2 isam(isam_params);

    // Create the factor graph and values object that will store new factors and values to add to the incremental graph
    NonlinearFactorGraph new_factors;
    Values new_values;  // values storing the initial estimates of new nodes in the factor graph
    /// Main loop:
    /// (1) we read the measurements
    /// (2) we create the corresponding factors in the graph
    /// (3) we solve the graph to obtain and optimal estimate of robot trajectory
    printf("-- Starting main loop: inference is performed at each time step, but we plot trajectory every 10 steps\n");
    size_t j = 0;
    for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; i++) {
        // At each non=IMU measurement we initialize a new node in the graph
        auto current_pose_key = X(i+1);
        auto current_vel_key = V(i+1);
        auto current_bias_key = B(i+1);
        double t = gps_measurements[i].time;

        if (i == first_gps_pose) {
            // Create initial estimate and prior on initial pose, velocity, and biases
            auto current_pose_key1= X(i);
            auto current_vel_key1 = V(i);
            auto current_bias_key1 = B(i);

            new_values.insert(current_pose_key1, current_pose_global);
            new_values.insert(current_vel_key1, current_velocity_global);
            new_values.insert(current_bias_key1, current_bias);
            new_values.insert(current_pose_key, current_pose_global);
            new_values.insert(current_vel_key, current_velocity_global);
            new_values.insert(current_bias_key, current_bias);

            new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key1, current_pose_global, sigma_init_x);
            new_factors.emplace_shared<PriorFactor<Vector3>>(current_vel_key1, current_velocity_global, sigma_init_v);
            new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key1, current_bias, sigma_init_b);
            new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, current_pose_global2, sigma_init_x);
            new_factors.emplace_shared<PriorFactor<Vector3>>(current_vel_key, current_velocity_global, sigma_init_v);
            new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(current_bias_key, current_bias, sigma_init_b);
        } 
        else {
            double t_previous = gps_measurements[i-1].time;

            // Summarize IMU data between the previous GPS measurement and now
            current_summarized_measurement = std::make_shared<PreintegratedImuMeasurements>(imu_params, current_bias);
            static size_t included_imu_measurement_count = 0;
            while (j < imu_measurements.size() && imu_measurements[j].time <= t) {
                if (imu_measurements[j].time >= t_previous) {
                    current_summarized_measurement->integrateMeasurement(imu_measurements[j].accelerometer,
                                                                         imu_measurements[j].gyroscope,
                                                                         imu_measurements[j].dt);
                    included_imu_measurement_count++;
                }
                j++;
            }
            
            printf("current_summarized_measurement!!!!!!");
            current_summarized_measurement->print();

            // Create IMU factor
            auto previous_pose_key = X(i);
            auto previous_vel_key = V(i);
            auto previous_bias_key = B(i);

            new_factors.emplace_shared<ImuFactor>(previous_pose_key, previous_vel_key,
                                                  current_pose_key, current_vel_key,
                                                  previous_bias_key, *current_summarized_measurement);

            // Bias evolution as given in the IMU metadata
            auto sigma_between_b = noiseModel::Diagonal::Sigmas((Vector6() <<
                   Vector3::Constant(sqrt(included_imu_measurement_count) * accelerometer_bias_sigma),
                   Vector3::Constant(sqrt(included_imu_measurement_count) * gyroscope_bias_sigma))
                 .finished());
            new_factors.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(previous_bias_key,
                                                                             current_bias_key,
                                                                             imuBias::ConstantBias(),
                                                                             sigma_between_b);

            // Create GPS factor
            cout << "checkchek" <<endl;
            Vector4 current_rotation = imuapproximate(gps_measurements[i].time, imu_measurements);
            
            // if there is no more IMU for orientation approximation
            if ((fabs(current_rotation[0] - 0)< 0.00000001) && (fabs(current_rotation[1] - 0)< 0.00000001) && (fabs(current_rotation[2] - 0)< 0.00000001) && (fabs(current_rotation[3] - 0)< 0.00000001)){
                break;
            }
            /*
            if ((fabs(current_rotation.x() - 0)< 0.00000001) && (fabs(current_rotation.y() - 0)< 0.00000001) && (fabs(current_rotation.z() - 0)< 0.00000001) && (fabs(current_rotation.w() - 1)< 0.00000001)){
                    break;
                }
            */
            //cout<< "current_rotation: " << current_rotation << endl;

            auto gps_pose = Pose3(Rot3(Quaternion(current_rotation[3], current_rotation[0], current_rotation[1], current_rotation[2])), gps_measurements[i].position);

            //if ((i % gps_skip) == 0) {
            new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, gps_pose, noise_model_gps);
            new_values.insert(current_pose_key, gps_pose);

            printf("################ POSE INCLUDED AT TIME %lf ################\n", t);
            cout << gps_pose.translation();
                printf("\n\n");
            //} else {
            //    new_values.insert(current_pose_key, current_pose_global);
            //}

            // Add initial values for velocity and bias based on the previous estimates
            new_values.insert(current_vel_key, current_velocity_global);
            new_values.insert(current_bias_key, current_bias);

            // Update solver
            // =======================================================================
            // We accumulate 2*GPSskip GPS measurements before updating the solver at
            // first so that the heading becomes observable.
            
            //if (i > (first_gps_pose + 2*gps_skip)) {
            //    printf("################ NEW FACTORS AT TIME %lf ################\n", t);
            //    new_factors.print();

            //    isam.update(new_factors, new_values);

                // Reset the newFactors and newValues list
             //   new_factors.resize(0);
             //   new_values.clear();

                // Extract the result/current estimates
             //   Values result = isam.calculateEstimate();

              //  current_pose_global = result.at<Pose3>(current_pose_key);
             //   current_velocity_global = result.at<Vector3>(current_vel_key);
              //  current_bias = result.at<imuBias::ConstantBias>(current_bias_key);

              //  printf("\n################ POSE AT TIME %lf ################\n", t);
              //  current_pose_global.print();
              //  printf("\n\n");
            //}
        }
    }

  
    LevenbergMarquardtOptimizer optimizer(new_factors, new_values);
    Values result = optimizer.optimize();
  
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
    //Marginals marginals(graph, result);
  
    // Save results to file
    printf("\nWriting results to file...\n");
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m)\n");
  


    //Values result = isam.calculateEstimate();
    for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; i++) {
        auto pose_key = X(i);
        auto vel_key = V(i);
        auto bias_key = B(i);

        auto pose = result.at<Pose3>(pose_key);
        auto velocity = result.at<Vector3>(vel_key);
        auto bias = result.at<imuBias::ConstantBias>(bias_key);

        auto pose_quat = pose.rotation().toQuaternion();
        auto gps = gps_measurements[i].position;

        cout << "State at #" << i << endl;
        cout << "Pose:" << endl << pose << endl;
        cout << "Velocity:" << endl << velocity << endl;
        cout << "Bias:" << endl << bias << endl;

        fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                gps_measurements[i].time,
                pose.x(), pose.y(), pose.z(),
                pose_quat.x(), pose_quat.y(), pose_quat.z(), pose_quat.w(),
                gps(0), gps(1), gps(2));
    }

    fclose(fp_out);
}


