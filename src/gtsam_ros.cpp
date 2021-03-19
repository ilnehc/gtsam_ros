/* ----------------------------------------------------------------------------
 * Copyright 2020, Li Chen <ilnehc@umich.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gtsam_ros.cpp
 *  @author Li Chen
 *  @brief  Source file for a ROS wrapper of the GTSAM
 *  @date   August 19, 2020
 **/

#include "gtsam_ros.h"
using namespace std;

// Constructor
GTSAM_ROS::GTSAM_ROS(ros::NodeHandle n) : n_(n) {}

// Initialize ROS node and gtsam core
void GTSAM_ROS::init() {
    // Create private node handle
    ros::NodeHandle nh("~");

    // Set noise parameters
    gtsam::Vector12 calibration = gtsam::Vector12::Zero();
    double std, cov;
    nh.param<double>("noise/accelerometer_std", std, 0.01);
    init_params_.setAccelerometerNoise(std);
    calibration(6) = std;
    nh.param<double>("noise/gyroscope_std", std, 0.000175);
    init_params_.setGyroscopeNoise(std);
    calibration(7) = std;
    nh.param<double>("noise/integration_std", std, 0);
    init_params_.setIntegrationNoise(std);
    calibration(8) = std;
    nh.param<double>("noise/accelerometer_bias_std", std, 0.000167);
    init_params_.setAccelerometerBiasNoise(std);
    calibration(9) = std;
    nh.param<double>("noise/gyroscope_bias_std", std, 2.91e-006);
    init_params_.setGyroscopeBiasNoise(std);
    calibration(10) = std;
    nh.param<double>("noise/average_delta_t", std, 0.0100395199348279);
    init_params_.setAverageDeltaT(std);
    calibration(11) = std;
    nh.param<double>("noise/landmark_std", std, 0.1);
    init_params_.setLandmarkNoise(std);
    nh.param<double>("noise/gps_std", std, 0.3);
    init_params_.setGpsNoise(std);
    //filter_.setNoiseParams(params);
/*
    // Set initial state and covariance
    RobotState state;
    Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg_init = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_init = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,15,15> P_init = Eigen::Matrix<double,15,15>::Zero();

    vector<double> param_vec;
    if (nh.getParam("prior/base_orientation", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 4);
        Eigen::Quaternion<double> q(param_vec[0], param_vec[1], param_vec[2], param_vec[3]);
        q.normalize();
        cur_baselink_orientation_ << q.x(),q.y(),q.z(),q.w();
        R_init = q.toRotationMatrix();
        initial_R_ = R_init;
    }
    if (nh.getParam("prior/base_velocity", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        v_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/base_position", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        p_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/gyroscope_bias", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        bg_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/accelerometer_bias", param_vec)) { 
        ROS_ASSERT(param_vec.size() == 3);
        ba_init << param_vec[0], param_vec[1], param_vec[2];
    }
    if (nh.getParam("prior/base_orientation_std", std)) { 
        cov = std*std;
        P_init(0,0) = cov; P_init(1,1) = cov; P_init(2,2) = cov;
    }
    if (nh.getParam("prior/base_velocity_std", std)) { 
        cov = std*std;
        P_init(3,3) = cov; P_init(4,4) = cov; P_init(5,5) = cov;
    }
    if (nh.getParam("prior/base_position_std", std)) { 
        cov = std*std;
        P_init(6,6) = cov; P_init(7,7) = cov; P_init(8,8) = cov;
    }
    if (nh.getParam("prior/gyroscope_bias_std", std)) { 
        cov = std*std;
        P_init(9,9) = cov; P_init(10,10) = cov; P_init(11,11) = cov;
    }
    if (nh.getParam("prior/accelerometer_bias_std", std)) { 
        cov = std*std;
        P_init(12,12) = cov; P_init(13,13) = cov; P_init(14,14) = cov;
    }
    state.setRotation(R_init);
    state.setVelocity(v_init);
    state.setPosition(p_init);
    state.setGyroscopeBias(bg_init);
    state.setAccelerometerBias(ba_init);
    state.setP(P_init);
    filter_.setState(state);

    // Print out initialization
    cout << "Robot's state is initialized to: \n";
    cout << filter_.getState() << endl;
    cout << filter_.getNoiseParams() << endl;
*/
    gtsam::Vector3 position = gtsam::Vector3::Zero();
    Core_.initialize(calibration, position);
/*
    // Set prior landmarks if given
    mapIntVector3d prior_landmarks;
    XmlRpc::XmlRpcValue list;
    if (nh.getParam("prior/landmarks", list)) { 
        for (int i = 0; i < list.size(); ++i) {
            XmlRpc::XmlRpcValue landmark = list[i];
            ROS_ASSERT_MSG(landmark["id"].getType() == XmlRpc::XmlRpcValue::TypeInt, "Prior landmark ID is not an int.");
            int id = static_cast<int>(landmark["id"]);
            XmlRpc::XmlRpcValue position = landmark["position"];
            ROS_ASSERT_MSG(position.size()==3, "Prior landmark position does not have 3 elements.");
            Eigen::Vector3d p_wl;
            for (int j=0; j<3; ++j){
                ROS_ASSERT_MSG(position[i].getType() == XmlRpc::XmlRpcValue::TypeDouble, "Prior landmark position is not a double.");
                p_wl(j) = static_cast<double>(position[j]);
            }
            ROS_INFO("Adding prior landmark (ID, position): %d, [%f, %f, %f]", id, p_wl[0], p_wl[1], p_wl[2]);
            prior_landmarks.insert(pair<int,Eigen::Vector3d> (landmark["id"], p_wl)); 
        }
    }
    filter_.setPriorLandmarks(prior_landmarks);
*/

    // ----- Settings --------
    nh.param<string>("settings/map_frame_id", map_frame_id_, "/map");
    nh.param<bool>("settings/enable_landmarks", enable_landmarks_, false);
    nh.param<bool>("settings/enable_linkstates", enable_linkstates, false);
    output_gps_ = false;
    if (nh.getParam("settings/raw_gps_output_path", gps_file_path_)) { 
        output_gps_ = true;
    }

    // Create publishers visualization markers if requested
    nh.param<bool>("settings/publish_visualization_markers", publish_visualization_markers_, false);
    if (publish_visualization_markers_) {
        string visualization_markers_topic;
        nh.param<string>("settings/visualization_markers_topic", visualization_markers_topic, "/markers");
        visualization_pub_ = n_.advertise<visualization_msgs::MarkerArray>(visualization_markers_topic, 1000);
        ROS_INFO("Visualization topic publishing under %s.", visualization_markers_topic.c_str());
    }

}

// Process Data
void GTSAM_ROS::run() {
    // Subscribe to all publishers
    this->subscribe();

    // Start main processing thread
    isam2_thread_ = std::thread([this]{this->mainIsam2Thread();});
    output_thread_ = std::thread([this]{this->outputPublishingThread();});
    //rviz_thread_ = std::thread([this]{this->rvizPublishingThread();});
    ros::spin();
}    

// Subscribe to all publishers
void GTSAM_ROS::subscribe() {
    double start_sub_time = ros::Time::now().toSec();

    // Create private node handle to get topic names
    ros::NodeHandle nh("~");
    string imu_topic;
    nh.param<string>("settings/imu_topic", imu_topic, "/imu");

    // Retrieve imu frame_id 
    ROS_INFO("Waiting for IMU message...");
    sensor_msgs::Imu::ConstPtr imu_msg = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic);
    imu_frame_id_ = imu_msg->header.frame_id;

    Eigen::Quaterniond quat(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    quat.normalize();
    initial_R_ = quat.toRotationMatrix();
    Eigen::Vector3d euler = initial_R_.eulerAngles(0, 1, 2);
    initial_yaw_ = euler(2);
    //filter_.SetTfEnuOdo(euler);

    ROS_INFO("IMU message received. IMU frame is set to %s.", imu_frame_id_.c_str());
    
    // Retrieve gps frame_id 
    string gps_topic;
    nh.param<string>("settings/gps_topic", gps_topic, "/gps");
    ROS_INFO("Waiting for GPS message...");
    sensor_msgs::NavSatFix::ConstPtr gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gps_topic);
    gps_frame_id_ = gps_msg->header.frame_id;
    // TODO: Convert output from IMU frame to base frame 
    nh.param<string>("settings/base_frame_id", base_frame_id_, "base_link");
    ROS_INFO("Waiting for tf lookup between frames %s and %s...", gps_frame_id_.c_str(), base_frame_id_.c_str());
    tf::TransformListener listener;
    try {
        listener.waitForTransform(base_frame_id_, gps_frame_id_, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(base_frame_id_, gps_frame_id_, ros::Time(0), base_to_gps_transform_);
        ROS_INFO("Tranform between frames %s and %s was found.", gps_frame_id_.c_str(), base_frame_id_.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        base_to_gps_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), gps_frame_id_, base_frame_id_);
    }

    double x = base_to_gps_transform_.getOrigin().getX(), 
           y = base_to_gps_transform_.getOrigin().getY(), 
           z = base_to_gps_transform_.getOrigin().getZ();
    Og0_to_Ob0_ << x, y, z;
    
    initial_lla_ << gps_msg->latitude, 
                    gps_msg->longitude, 
                    gps_msg->altitude;
    if (output_gps_) {
        file.open(gps_file_path_.c_str());
        file << "timestamp [ns]" << "," << "odo x" << "," << "odo y" << "," << "odo z" << endl;
        file.close();
    }
    initial_ecef_ = lla_to_ecef(initial_lla_);

    ROS_INFO("GPS message received. GPS frame is set to %s.", gps_frame_id_.c_str());

    // Retrieve prior linkstates 
    string linkstates_topic = "";
    if (enable_linkstates) {
        nh.param<string>("settings/base_link_topic", linkstates_topic, ""); 
        gazebo_msgs::LinkStates::ConstPtr link_msg = ros::topic::waitForMessage<gazebo_msgs::LinkStates>(linkstates_topic);
        int n = (link_msg->pose).size();
        initial_linkstate_ << link_msg->pose[n-9].position.x,  // 8 is gps_link, 9 is base_link
                              link_msg->pose[n-9].position.y, 
                              link_msg->pose[n-9].position.z;
        ROS_INFO("Prior link states message received.");
    }

    // Retrieve camera frame_id and transformation between camera and imu
    /*
    string landmarks_topic;
    if (enable_landmarks_) {
        nh.param<string>("settings/landmarks_topic", landmarks_topic, "/landmarks");
        ROS_INFO("Waiting for Landmark message...");
        inekf_msgs::LandmarkArray::ConstPtr landmark_msg = ros::topic::waitForMessage<inekf_msgs::LandmarkArray>(landmarks_topic);
        string camera_frame_id = landmark_msg->header.frame_id;
        // apriltag_msgs::AprilTagDetectionArray::ConstPtr landmark_msg = ros::topic::waitForMessage<apriltag_msgs::AprilTagDetectionArray>(landmarks_topic);
        // while(landmark_msg->detections.size()==0) {
        //     landmark_msg = ros::topic::waitForMessage<apriltag_msgs::AprilTagDetectionArray>(landmarks_topic);
        //     if (landmark_msg->detections.size() != 0) 
        //         break;
        // }
        // string camera_frame_id = landmark_msg->detections[0].pose.header.frame_id;
        ROS_INFO("Landmark message received. Camera frame is set to %s.", camera_frame_id.c_str());

        ROS_INFO("Waiting for tf lookup between frames %s and %s...", camera_frame_id.c_str(), imu_frame_id_.c_str());
        tf::TransformListener listener;
        try {
            listener.waitForTransform(imu_frame_id_, camera_frame_id, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(imu_frame_id_, camera_frame_id, ros::Time(0), camera_to_imu_transform_);
            ROS_INFO("Tranform between frames %s and %s was found.", camera_frame_id.c_str(), imu_frame_id_.c_str());
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s. Using identity transform.",ex.what());
            camera_to_imu_transform_ = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), camera_frame_id, imu_frame_id_);
        }   
    }
    */

    // Subscribe to IMU publisher
    ROS_INFO("Subscribing to %s.", imu_topic.c_str());
    imu_sub_ = n_.subscribe(imu_topic, 1, &GTSAM_ROS::imuCallback, this);

    // Subscribe to GPS publisher
    ROS_INFO("Subscribing to %s.", gps_topic.c_str());
    gps_sub_ = n_.subscribe(gps_topic, 1, &GTSAM_ROS::gpsCallback, this);

    // Subscribe to link states publisher
    if (enable_linkstates) {
        ROS_INFO("Subscribing to %s.", linkstates_topic.c_str());
        linkstates_sub_ = n_.subscribe(linkstates_topic, 1000, &GTSAM_ROS::linkstatesCallback, this);
    }
    /*
    // Subscribe to Landmark publisher
    if (enable_landmarks_) {
        ROS_INFO("Subscribing to %s.", landmarks_topic.c_str());
        landmarks_sub_ = n_.subscribe(landmarks_topic, 1000, &GTSAM_ROS::landmarkCallback, this);
    }
    */

   double end_sub_time = ros::Time::now().toSec();
   ROS_INFO("subscribe start time: %f", start_sub_time);
   ROS_INFO("subscribe end time: %f", end_sub_time);
}

// IMU Callback function
void GTSAM_ROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new ImuMeasurement(msg));
    m_queue_.push(ptr);
}

// GPS Callback function
void GTSAM_ROS::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    shared_ptr<GpsMeasurement> ptr(new GpsMeasurement(msg));
    //m_queue_.push(ptr);

    geometry_msgs::PoseWithCovarianceStamped::Ptr pose_msg(new geometry_msgs::PoseWithCovarianceStamped());
    pose_msg->header.seq = msg->header.seq;
    pose_msg->header.stamp = msg->header.stamp;
    pose_msg->header.frame_id = base_frame_id_; 
    gtsam::Vector3 position = lla_to_enu(ptr->getData());
    pose_msg->pose.pose.position.x = position(0); 
    pose_msg->pose.pose.position.y = position(1); 
    pose_msg->pose.pose.position.z = position(2); 
    /*
    Eigen::Quaternion<double> orientation(state.getRotation());
    orientation.normalize();
    // Transform from imu frame to base frame
    tf::Transform imu_pose;
    imu_pose.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
    imu_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
    tf::Transform base_pose = imu_to_base_transform*imu_pose;
    tf::Quaternion base_orientation = base_pose.getRotation().normalize();
    tf::Vector3 base_position = base_pose.getOrigin();  
    // Construct message
    pose_msg->pose.pose.position.x = base_position.getX(); 
    pose_msg->pose.pose.position.y = base_position.getY(); 
    pose_msg->pose.pose.position.z = base_position.getZ(); 
    pose_msg->pose.pose.orientation.w = base_orientation.getW();
    pose_msg->pose.pose.orientation.x = base_orientation.getX();
    pose_msg->pose.pose.orientation.y = base_orientation.getY();
    pose_msg->pose.pose.orientation.z = base_orientation.getZ();
    Eigen::Matrix<double,6,6> P_pose; // TODO: convert covariance from imu to body frame (adjoint?)
    P_pose.block<3,3>(0,0) = P.block<3,3>(0,0);
    P_pose.block<3,3>(0,3) = P.block<3,3>(0,6);
    P_pose.block<3,3>(3,0) = P.block<3,3>(6,0);
    P_pose.block<3,3>(3,3) = P.block<3,3>(6,6);
    for (int i=0; i<36; ++i) {
        pose_msg->pose.covariance[i] = P_pose(i);
    }
    */

    shared_ptr<PoseMeasurement> pose_ptr(new PoseMeasurement(pose_msg));
    m_queue_.push(pose_ptr);

    //ros::Duration(2.0).sleep();
}

// Link States Callback function
void GTSAM_ROS::linkstatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new GtLinkMeasurement(msg, ros::Time::now().toSec()));
    m_queue_.push(ptr);
}
/*
// Landmark Callback function
void GTSAM_ROS::landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg) {
    shared_ptr<Measurement> ptr(new LandmarkMeasurement(msg, camera_to_imu_transform_));
    m_queue_.push(ptr);
}
*/
// reference to https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
gtsam::Vector3 GTSAM_ROS::lla_to_ecef(const gtsam::Vector3& lla) {
    const double equatorial_radius = 6378137.0;
    const double polar_radius = 6356752.31424518;
    const double square_ratio = pow(polar_radius,2) / pow(equatorial_radius,2);

    const double lat = lla(0) * M_PI / 180;
    const double lon = lla(1) * M_PI / 180;
    const double alt = lla(2);
    const double N = equatorial_radius / sqrt(1 - (1-square_ratio) * pow(sin(lat),2));

    const double z = (square_ratio * N + alt) * sin(lat);
    const double q = (N + alt) * cos(lat);
    const double x = q * cos(lon);
    const double y = q * sin(lon);

    return (gtsam::Vector3() << x, y, z).finished();
}

gtsam::Vector3 GTSAM_ROS::lla_to_enu(const gtsam::Vector3& lla) {
    // readings are geodetic
    gtsam::Vector3 cur_ecef = lla_to_ecef(lla);
    gtsam::Vector3 r_ecef = cur_ecef - initial_ecef_;

    double phi = initial_lla_(0) * M_PI / 180;
    double lam = initial_lla_(1) * M_PI / 180;

    gtsam::Matrix33 R = (gtsam::Matrix33() <<
        -sin(lam),          cos(lam),           0,
        -cos(lam)*sin(phi), -sin(lam)*sin(phi), cos(phi),
        cos(lam)*cos(phi),  sin(lam)*cos(phi),  sin(phi)
    ).finished();

    return R * r_ecef;
}



void GTSAM_ROS::mainIsam2Thread() {
    cout << "Inside Main Filtering Thread\n";
    shared_ptr<Measurement> m_ptr;
    shared_ptr<ImuMeasurement> imu_ptr_last;
    double t, t_last;
    bool first_gps = true;

    // Block until first IMU measurement is received
    while (ros::ok()) {
        // Try next item (Blocking)
        m_queue_.pop(m_ptr);
        if (m_ptr->getType() == IMU) {
            ROS_INFO("First IMU measurement received. Starting GTSAM.");
            double current_time = ros::Time::now().toSec();
            t_last = m_ptr->getTime();
            ROS_INFO("time delayed: %f", current_time - t_last);
            imu_ptr_last = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
    
    // Main loop
    while (ros::ok()) {
        // Throw warning if measurement queue is getting too large
        if (m_queue_.size() > MAX_QUEUE_SIZE) {
            ROS_WARN("Measurement queue size (%d) is greater than MAX_QUEUE_SIZE. GTSAM is not realtime!", m_queue_.size());
        }   
        // Wait until buffer is full
        while(m_queue_.size() < QUEUE_BUFFER_SIZE) {
            this_thread::sleep_for(chrono::milliseconds(10));
        }
        // Retrieve next measurement (Blocking)
        m_queue_.pop(m_ptr);
        // ROS_ERROR("Time: %f", m_ptr->getTime());
        // Handle measurement
        switch (m_ptr->getType()) {
            case IMU: {
                // ROS_INFO("Propagating state with IMU measurements.");
                auto imu_ptr = dynamic_pointer_cast<ImuMeasurement>(m_ptr);
                double current_time = ros::Time::now().toSec();
                t = imu_ptr->getTime();
                ROS_INFO("time delayed: %f", current_time - t);
                //filter_.Propagate(imu_ptr_last->getData(), t - t_last);
                Core_.addIMU(imu_ptr);
                t_last = t;
                imu_ptr_last = imu_ptr;

                // Uncomment out below part to use IMU orientation measurement to compute gps-base transform 

                cur_baselink_orientation_ = imu_ptr->getOri();

                break;
            }
            case GPS: {
                // ROS_INFO("Correcting state with GPS measurements.");
                // Uncomment out this part to use GPS measurement
                auto gps_ptr = dynamic_pointer_cast<GpsMeasurement>(m_ptr);

                Eigen::Matrix<double,3,1> gps_xyz = lla_to_enu(gps_ptr->getData());
                Eigen::Vector3d position = gps_xyz.head(3); //state.getPosition();
                //filter_.CorrectGPS(position);
                if (output_gps_) {
                    file.open(gps_file_path_.c_str(), ios::app);
                    file.precision(16);
                    double t = gps_ptr->getTime();
	                size_t t1 = t * 1e9;
                    file << t1 << "," << position(0) << "," << position(1) << "," << position(2) << endl;
                    file.close();
                }

                //RobotState state = filter_.getState();
                //position = state.getPosition();
                //Eigen::Quaternion<double> orientation(state.getRotation());
                Eigen::Quaternion<double> orientation(cur_baselink_orientation_[3],cur_baselink_orientation_[0],cur_baselink_orientation_[1],cur_baselink_orientation_[2]);
                orientation.normalize();
                //orientation.x() = 0;
                //orientation.y() = 0;
                
                //position -= Og0_to_Ob0_;
                //std::cout << "Current state orientation: " << orientation.vec() << std::endl;
                //std::cout << "Current gps position after lla_to_enu: " << position << std::endl;
                // Transform from gps frame to base frame
                tf::Transform gps_pose;

                // Method1: set GPS orientation
                /*
                gps_pose.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
                gps_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
                tf::Transform base_pose = gps_pose*base_to_gps_transform_.inverse(); // in initial gps frame
                */

                // Method2: zero out rotation in GPS orientation, add the rotation in offset
                // Methametically, these two are same
                
                gps_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
                gps_pose.setRotation( tf::Quaternion::getIdentity() );
                tf::Transform gps_offset_rotated;
                gps_offset_rotated.setOrigin( tf::Vector3(-Og0_to_Ob0_(0),-Og0_to_Ob0_(1),-Og0_to_Ob0_(2)) );
                gps_offset_rotated.setOrigin( tf::quatRotate(tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()), gps_offset_rotated.getOrigin()) );
                gps_offset_rotated.setRotation( tf::Quaternion::getIdentity() );
                tf::Transform base_pose = gps_offset_rotated.inverse() * gps_pose;
                

                tf::Vector3 base_position = base_pose.getOrigin();
                Eigen::Vector3d base_Ob(base_position.getX(),base_position.getY(),base_position.getZ());
                base_Ob -= initial_R_*Og0_to_Ob0_; // subtract origin transition
                //std::cout << "Here is the diff between gps_xyz and base_ob: " << base_Ob-position << std::endl;
                //cout << "[" << position(0) << "," << position(1) << "," << position(2) << "]" << endl;
                
                
                //filter_.CorrectGPS(base_Ob);
                break;
            }
            case POSE: {
                auto pose_ptr = dynamic_pointer_cast<PoseMeasurement>(m_ptr);
/*
                if (output_gps_) {
                    file.open(gps_file_path_.c_str(), ios::app);
                    file.precision(16);
                    double t = pose_ptr->getTime();
	                size_t t1 = t * 1e9;
                    Eigen::Vector3d position = pose_ptr->getData();
                    file << t1 << "," << position(0) << "," << position(1) << "," << position(2) << endl;
                    file.close();
                }
*/
                // transform before add to graph
                gtsam::Pose3 pose = Core_.getCurPose();
                gtsam::Quaternion quat = pose.rotation().toQuaternion();
                Eigen::Vector3d position = pose_ptr->getData();
                tf::Transform gps_pose;
                if (first_gps) {
                    gps_pose.setRotation( tf::Quaternion(
                        cur_baselink_orientation_[0],cur_baselink_orientation_[1],cur_baselink_orientation_[2],cur_baselink_orientation_[3]) );
                    first_gps = false;
                }
                else {
                    gps_pose.setRotation( tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()) );
                }
                gps_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
                tf::Transform base_pose = gps_pose * base_to_gps_transform_.inverse();
                tf::Quaternion base_orientation = base_pose.getRotation().normalize();
                tf::Vector3 base_position = base_pose.getOrigin();
                Eigen::Vector3d base_Ob(base_position.getX(),base_position.getY(),base_position.getZ());
                base_Ob += initial_R_*Og0_to_Ob0_; // subtract origin transition
                
                geometry_msgs::PoseWithCovarianceStamped::Ptr pose_msg(new geometry_msgs::PoseWithCovarianceStamped());
                ros::Time measurement_time(pose_ptr->getTime());
                pose_msg->header.stamp = measurement_time;
                pose_msg->header.frame_id = base_frame_id_; 
                pose_msg->pose.pose.position.x = base_Ob(0); 
                pose_msg->pose.pose.position.y = base_Ob(1); 
                pose_msg->pose.pose.position.z = base_Ob(2); 
                shared_ptr<PoseMeasurement> posePtr(new PoseMeasurement(pose_msg));

                if (output_gps_) {
                    file.open(gps_file_path_.c_str(), ios::app);
                    file.precision(16);
                    double t = pose_ptr->getTime();
	                size_t t1 = t * 1e9;
                    file << t1 << "," << base_Ob(0) << "," << base_Ob(1) << "," << base_Ob(2) << endl;
                    file.close();
                }

                Core_.addGPS(posePtr);

                //Core_.addGPS(pose_ptr);
                break;
            }
            case LINK: {
                // Uncomment out below part to use ground truth base link states as GPS measurement
                /*
                auto link_ptr = dynamic_pointer_cast<GtLinkMeasurement>(m_ptr);
                Eigen::Vector3d base_Ob(link_ptr->getPos() - initial_linkstate_);

                if (output_gps_) {
                    file.open(gps_file_path_.c_str(), ios::app);
                    file.precision(16);
                    file << link_ptr->getTime() << "," << base_Ob(0) << "," << base_Ob(1) << "," << base_Ob(2) << endl;
                    file.close();
                }
                
                filter_.CorrectGPS(base_Ob);
                */

                // Uncomment out below part to use ground truth base pose to compute gps-base transform 
                
                auto link_ptr = dynamic_pointer_cast<GtLinkMeasurement>(m_ptr);
                //cur_baselink_orientation_ = link_ptr->getOri();

                break;
            }
            /*
            case LANDMARK: {
                // ROS_INFO("Correcting state with LANDMARK measurements.");
                auto landmark_ptr = dynamic_pointer_cast<LandmarkMeasurement>(m_ptr);
                filter_.CorrectLandmarks(landmark_ptr->getData());
                if (publish_visualization_markers_) {
                    this->publishLandmarkMeasurementMarkers(landmark_ptr);
                }
                break;
            }*/
            default:
                cout << "Unknown measurement, skipping...\n";
        }
    }
}

/*
// Publish line markers between IMU and detected landmarks
void GTSAM_ROS::publishLandmarkMeasurementMarkers(shared_ptr<LandmarkMeasurement> ptr){
    visualization_msgs::MarkerArray markers_msg;
    visualization_msgs::Marker landmark_measurement_msg;
    landmark_measurement_msg.header.frame_id = map_frame_id_;
    landmark_measurement_msg.header.stamp = ros::Time(ptr->getTime());
    landmark_measurement_msg.header.seq = 0;
    landmark_measurement_msg.ns = "landmark_measurements";
    landmark_measurement_msg.type = visualization_msgs::Marker::LINE_LIST;
    landmark_measurement_msg.action = visualization_msgs::Marker::ADD;
    landmark_measurement_msg.id = 0;
    landmark_measurement_msg.scale.x = 0.01;
    landmark_measurement_msg.scale.y = 0.01;
    landmark_measurement_msg.scale.z = 0.01;
    landmark_measurement_msg.color.a = 1.0; // Don't forget to set the alpha!
    landmark_measurement_msg.color.r = 0.0;
    landmark_measurement_msg.color.g = 0.0;
    landmark_measurement_msg.color.b = 1.0;

    geometry_msgs::Point base_point, landmark_point;
    RobotState state = filter_.getState();
    Eigen::MatrixXd X = state.getX();
    Eigen::Vector3d position = state.getPosition();
    base_point.x = position(0);
    base_point.y = position(1);
    base_point.z = position(2);

    vectorLandmarks measured_landmarks = ptr->getData();
    mapIntVector3d prior_landmarks = filter_.getPriorLandmarks();
    map<int,int> estimated_landmarks = filter_.getEstimatedLandmarks();
    for (auto it=measured_landmarks.begin(); it!=measured_landmarks.end(); ++it) {
        // Search through prior landmarks
        auto search_prior = prior_landmarks.find(it->id);
        if (search_prior != prior_landmarks.end()) {
            landmark_point.x = search_prior->second(0);
            landmark_point.y = search_prior->second(1);
            landmark_point.z = search_prior->second(2);
            landmark_measurement_msg.points.push_back(base_point);
            landmark_measurement_msg.points.push_back(landmark_point);
            continue;
        }
        // Search through estimated landmarks
        auto search_estimated = estimated_landmarks.find(it->id);
        if (search_estimated != estimated_landmarks.end()) {
            landmark_point.x = X(0,search_estimated->second);
            landmark_point.y = X(1,search_estimated->second);
            landmark_point.z = X(2,search_estimated->second);
            landmark_measurement_msg.points.push_back(base_point);
            landmark_measurement_msg.points.push_back(landmark_point);
            continue;
        }
    }
    // Publish
    markers_msg.markers.push_back(landmark_measurement_msg);
    visualization_pub_.publish(markers_msg);
}
*/

// Thread for publishing the output of the filter
void GTSAM_ROS::outputPublishingThread() {
    // Create private node handle to get topic names
    ros::NodeHandle nh("~");
    double publish_rate;
    string base_frame_id, pose_topic, state_topic, path_topic, path_frame_id;
    string pose_topic4Nav;
    string offset_x, offset_y;

    nh.param<double>("settings/publish_rate", publish_rate, 10);
    nh.param<string>("settings/base_frame_id", base_frame_id, "/imu");
    nh.param<string>("settings/pose_topic", pose_topic, "/pose");
    nh.param<string>("settings/pose_topic4Nav", pose_topic4Nav, "/pose");
    nh.param<string>("settings/state_topic", state_topic, "/state");
    nh.param<string>("settings/path_topic", path_topic, "/path");
    nh.param<string>("settings/path_frame_id", path_frame_id, "path");

    nh.param<string>("settings/offset_x", offset_x, "offx");
    nh.param<string>("settings/offset_y", offset_y, "offy");

    ROS_INFO("Map frame id set to %s.", map_frame_id_.c_str());
    ROS_INFO("Base frame id set to %s.", base_frame_id.c_str());
    ROS_INFO("Path frame id set to %s.", path_frame_id.c_str());
    ROS_INFO("Pose topic publishing under %s.", pose_topic.c_str());
    ROS_INFO("Pose topic Nav publishing under %s.", pose_topic4Nav.c_str());
    ROS_INFO("State topic publishing under %s.", state_topic.c_str());
    ROS_INFO("Path topic publishing under %s.", path_topic.c_str());

    ROS_INFO("offset_x %s.", offset_x.c_str());
    ROS_INFO("offset_y %s.", offset_y.c_str());

    // TODO: Convert output from IMU frame to base frame 
    ROS_INFO("Waiting for tf lookup between frames %s and %s...", imu_frame_id_.c_str(), base_frame_id.c_str());
    tf::TransformListener listener;
    tf::StampedTransform imu_to_base_transform;
    try {
        listener.waitForTransform(base_frame_id, imu_frame_id_, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(base_frame_id, imu_frame_id_, ros::Time(0), imu_to_base_transform);
        ROS_INFO("Tranform between frames %s and %s was found.", imu_frame_id_.c_str(), base_frame_id.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        imu_to_base_transform = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), imu_frame_id_, base_frame_id);
    } 

    // Create publishers for pose and state messages
    ros::Publisher pose_pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    ros::Publisher poseNav_pub = n_.advertise<nav_msgs::Odometry>(pose_topic4Nav, 1000);
    ros::Publisher path_pub = n_.advertise<nav_msgs::Path>(path_topic, 1000);
    //ros::Publisher state_pub = n_.advertise<inekf_msgs::State>(state_topic, 1000);
    static tf::TransformBroadcaster tf_broadcaster;
    
    // Init loop params
    uint32_t seq = 0;
    geometry_msgs::Point point_last;
    ros::Rate loop_rate(publish_rate);

    // Main loop
    while(true) {
        gtsam::Pose3 pose = Core_.getCurPose();
        gtsam::Vector6 twist = Core_.getTwist();
        /*
        RobotState state = filter_.getState();
        Eigen::MatrixXd X = state.getX();
        Eigen::MatrixXd P = state.getP();
        */
        
        // Create and send pose message
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.seq = seq;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = map_frame_id_;
        Eigen::Vector3d position = pose.translation().vector();
        position(0) += std::stof(offset_x);
        position(1) += std::stof(offset_y);
        // position(0) += 157.39;
        // position(1) += 107.58;
        gtsam::Quaternion quat = pose.rotation().toQuaternion();
         
        pose_msg.pose.pose.position.x = position(0); 
        pose_msg.pose.pose.position.y = position(1); 
        pose_msg.pose.pose.position.z = position(2);
        pose_msg.pose.pose.orientation.x = quat.x();
        pose_msg.pose.pose.orientation.y = quat.y();
        pose_msg.pose.pose.orientation.z = quat.z();
        pose_msg.pose.pose.orientation.w = quat.w();

        // Create and send pose msg for nav // pose here is current one
        nav_msgs::Odometry OdomPose_msg;
        OdomPose_msg.header.seq = seq;
        OdomPose_msg.header.stamp = ros::Time::now();
        OdomPose_msg.header.frame_id = map_frame_id_;

        // Eigen::Vector3d position = pose.translation().vector();
        // gtsam::Quaternion quat = pose.rotation().toQuaternion();

        // let covariance blank here
         
        OdomPose_msg.pose.pose.position.x = position(0); 
        OdomPose_msg.pose.pose.position.y = position(1); 
        OdomPose_msg.pose.pose.position.z = position(2);
        OdomPose_msg.pose.pose.orientation.x = quat.x();
        OdomPose_msg.pose.pose.orientation.y = quat.y();
        OdomPose_msg.pose.pose.orientation.z = quat.z();
        OdomPose_msg.pose.pose.orientation.w = quat.w();

        OdomPose_msg.twist.twist.linear.x = twist(3);
        OdomPose_msg.twist.twist.linear.y = twist(4);
        OdomPose_msg.twist.twist.linear.z = twist(5);
        OdomPose_msg.twist.twist.angular.x = twist(0);
        OdomPose_msg.twist.twist.angular.y = twist(1);
        OdomPose_msg.twist.twist.angular.z = twist(2);

/*        
        tf::Transform gps_pose;
        gps_pose.setRotation( tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()) );
        gps_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
        tf::Transform base_pose = gps_pose * base_to_gps_transform_.inverse();
        tf::Quaternion base_orientation = base_pose.getRotation().normalize();
        tf::Vector3 base_position = base_pose.getOrigin();
        Eigen::Vector3d base_Ob(base_position.getX(),base_position.getY(),base_position.getZ());
        base_Ob += initial_R_*Og0_to_Ob0_; // subtract origin transition
        pose_msg.pose.pose.position.x = base_Ob(0); 
        pose_msg.pose.pose.position.y = base_Ob(1); 
        pose_msg.pose.pose.position.z = base_Ob(2);
        pose_msg.pose.pose.orientation.x = base_orientation.getX();
        pose_msg.pose.pose.orientation.y = base_orientation.getY();
        pose_msg.pose.pose.orientation.z = base_orientation.getZ();
        pose_msg.pose.pose.orientation.w = base_orientation.getW(); 
*/
        geometry_msgs::PoseStamped final_pose_msg;
        path.header.seq = seq;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = path_frame_id; 
        final_pose_msg.pose.position.x = position(0); 
        final_pose_msg.pose.position.y = position(1); 
        final_pose_msg.pose.position.z = position(2);
        final_pose_msg.pose.orientation.x = quat.x();
        final_pose_msg.pose.orientation.y = quat.y();
        final_pose_msg.pose.orientation.z = quat.z();
        final_pose_msg.pose.orientation.w = quat.w();
        path.poses.push_back(final_pose_msg);
        /*
        Eigen::Quaternion<double> orientation(state.getRotation());
        orientation.normalize();
        // Transform from imu frame to base frame
        tf::Transform imu_pose;
        imu_pose.setRotation( tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w()) );
        imu_pose.setOrigin( tf::Vector3(position(0),position(1),position(2)) );
        tf::Transform base_pose = imu_to_base_transform*imu_pose;
        tf::Quaternion base_orientation = base_pose.getRotation().normalize();
        tf::Vector3 base_position = base_pose.getOrigin();  
        // Construct message
        pose_msg.pose.pose.position.x = base_position.getX(); 
        pose_msg.pose.pose.position.y = base_position.getY(); 
        pose_msg.pose.pose.position.z = base_position.getZ(); 
        pose_msg.pose.pose.orientation.w = base_orientation.getW();
        pose_msg.pose.pose.orientation.x = base_orientation.getX();
        pose_msg.pose.pose.orientation.y = base_orientation.getY();
        pose_msg.pose.pose.orientation.z = base_orientation.getZ();
        Eigen::Matrix<double,6,6> P_pose; // TODO: convert covariance from imu to body frame (adjoint?)
        P_pose.block<3,3>(0,0) = P.block<3,3>(0,0);
        P_pose.block<3,3>(0,3) = P.block<3,3>(0,6);
        P_pose.block<3,3>(3,0) = P.block<3,3>(6,0);
        P_pose.block<3,3>(3,3) = P.block<3,3>(6,6);
        for (int i=0; i<36; ++i) {
            pose_msg.pose.covariance[i] = P_pose(i);
        }
        */
        pose_pub.publish(pose_msg);
        path_pub.publish(path);
        poseNav_pub.publish(OdomPose_msg);

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "wamv/odom";
        transformStamped.child_frame_id = "wamv/base_link";
        transformStamped.transform.translation.x = position(0);
        transformStamped.transform.translation.y = position(1); 
        transformStamped.transform.translation.z = position(2); 
        
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();
      
        br.sendTransform(transformStamped);
        /*
        // Create and send tf message
        tf_broadcaster.sendTransform(tf::StampedTransform(base_pose, ros::Time::now(), map_frame_id_, base_frame_id));

        // Create and send State message
        inekf_msgs::State state_msg;
        state_msg.header.seq = seq;
        state_msg.header.stamp = ros::Time::now();
        state_msg.header.frame_id = map_frame_id_; 
        state_msg.pose = pose_msg.pose.pose;
        Eigen::Vector3d velocity = state.getVelocity();
        state_msg.velocity.x = velocity(0); 
        state_msg.velocity.y = velocity(1); 
        state_msg.velocity.z = velocity(2); 
        map<int,int> estimated_landmarks = filter_.getEstimatedLandmarks();
        for (auto it=estimated_landmarks.begin(); it!=estimated_landmarks.end(); ++it) {
            inekf_msgs::Landmark landmark;
            landmark.id = it->first;
            landmark.position.x = X(0,it->second);
            landmark.position.y = X(1,it->second);
            landmark.position.z = X(2,it->second);
            state_msg.landmarks.push_back(landmark);
        }
        Eigen::Vector3d bg = state.getGyroscopeBias();
        state_msg.gyroscope_bias.x = bg(0); 
        state_msg.gyroscope_bias.y = bg(1); 
        state_msg.gyroscope_bias.z = bg(2); 
        Eigen::Vector3d ba = state.getAccelerometerBias();
        state_msg.accelerometer_bias.x = ba(0); 
        state_msg.accelerometer_bias.y = ba(1); 
        state_msg.accelerometer_bias.z = ba(2); 
        state_pub.publish(state_msg);

        // Create and send markers for visualization
        if (publish_visualization_markers_) {
            visualization_msgs::MarkerArray markers_msg;

            // Add prior landmarks
            mapIntVector3d prior_landmarks = filter_.getPriorLandmarks();
            for (auto it=prior_landmarks.begin(); it!=prior_landmarks.end(); ++it) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = map_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.header.seq = seq;
                marker.ns = "prior_landmarks";
                marker.id = it->first;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = it->second(0);
                marker.pose.position.y = it->second(1);
                marker.pose.position.z = it->second(2);
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.lifetime = ros::Duration(1.0/publish_rate);
                markers_msg.markers.push_back(marker);
            }

            // Add estimated landmarks
            for (auto it=estimated_landmarks.begin(); it!=estimated_landmarks.end(); ++it) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = map_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.header.seq = seq;
                marker.ns = "estimated_landmarks";
                marker.id = it->first;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = X(0,it->second);
                marker.pose.position.y = X(1,it->second);
                marker.pose.position.z = X(2,it->second);
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = sqrt(P(3+3*(it->second-3),3+3*(it->second-3)));
                marker.scale.y = sqrt(P(4+3*(it->second-3),4+3*(it->second-3)));
                marker.scale.z = sqrt(P(5+3*(it->second-3),5+3*(it->second-3)));
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = ros::Duration(1.0/publish_rate);
                markers_msg.markers.push_back(marker);
            }

            // Add trajectory
            visualization_msgs::Marker traj_marker;
            traj_marker.header.frame_id = map_frame_id_;
            traj_marker.header.stamp = ros::Time();
            traj_marker.header.seq = seq;
            traj_marker.ns = "trajectory";
            traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
            traj_marker.action = visualization_msgs::Marker::ADD;
            traj_marker.id = seq;
            traj_marker.scale.x = 0.01;
            traj_marker.color.a = 1.0; // Don't forget to set the alpha!
            traj_marker.color.r = 1.0;
            traj_marker.color.g = 0.0;
            traj_marker.color.b = 0.0;
            traj_marker.lifetime = ros::Duration(100.0);
            geometry_msgs::Point point;
            point.x = position(0);
            point.y = position(1);
            point.z = position(2);
            if (seq>0){
                traj_marker.points.push_back(point_last);
                traj_marker.points.push_back(point);
                markers_msg.markers.push_back(traj_marker);
            }   
            point_last = point;

            // Publish markers
            visualization_pub_.publish(markers_msg);
        }
    */
        seq++;
        loop_rate.sleep();
    }

}


void GTSAM_ROS::rvizPublishingThread() {

    ros::NodeHandle nh("~");
    double publish_rate = 5;
    
    string base_frame_id, pose_topic, state_topic;
    /*
    nh.param<double>("settings/publish_rate", publish_rate, 10);
    nh.param<string>("settings/base_frame_id", base_frame_id, "/imu");
    nh.param<string>("settings/pose_topic", pose_topic, "/pose");
    nh.param<string>("settings/state_topic", state_topic, "/state");
    
    ROS_INFO("Map frame id set to %s.", map_frame_id_.c_str());
    ROS_INFO("Base frame id set to %s.", base_frame_id.c_str());
    ROS_INFO("Pose topic publishing under %s.", pose_topic.c_str());
    ROS_INFO("State topic publishing under %s.", state_topic.c_str());

    // TODO: Convert output from IMU frame to base frame 
    ROS_INFO("Waiting for tf lookup between frames %s and %s...", imu_frame_id_.c_str(), base_frame_id.c_str());
    tf::TransformListener listener;
    tf::StampedTransform imu_to_base_transform;
    try {
        listener.waitForTransform(base_frame_id, imu_frame_id_, ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform(base_frame_id, imu_frame_id_, ros::Time(0), imu_to_base_transform);
        ROS_INFO("Tranform between frames %s and %s was found.", imu_frame_id_.c_str(), base_frame_id.c_str());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s. Using identity transform.",ex.what());
        imu_to_base_transform = tf::StampedTransform( tf::Transform::getIdentity(), ros::Time::now(), imu_frame_id_, base_frame_id);
    } 

    // Create publishers for pose and state messages
    ros::Publisher pose_pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    //ros::Publisher state_pub = n_.advertise<inekf_msgs::State>(state_topic, 1000);
    static tf::TransformBroadcaster tf_broadcaster;
    */
    // Init loop params
    uint32_t seq = 0;
    geometry_msgs::Point point_last;
    ros::Rate loop_rate(publish_rate);

    while (true) {
        gtsam::Values result = Core_.getResult();
        gtsam::KeyVector kv = result.keys();
        printf("################ KeyVector ################\n");
        //gtsam::PrintKeyVector(kv);

        loop_rate.sleep();
    }
    
}