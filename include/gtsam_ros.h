/* ----------------------------------------------------------------------------
 * Copyright 2020, Li Chen <ilnehc@umich.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gtsam_ros.h
 *  @author Li Chen
 *  @brief  Header file for a ROS wrapper of the GTSAM
 *  @date   August 19, 2020
 **/

#ifndef GTSAM_ROS_H
#define GTSAM_ROS_H 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <boost/lockfree/queue.hpp>
#include "gtsam_core.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "Measurement.h"
#include "Queue.h"
#include "NoiseParams.h"
#include "visualization_msgs/MarkerArray.h"
#include <mutex>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// !!!! BE CAREFUL !!!!
#define QUEUE_BUFFER_SIZE 1
#define MAX_QUEUE_SIZE 150


class GTSAM_ROS {
    public:
        GTSAM_ROS(ros::NodeHandle n);
        void init();
        void run();

    private: 
        ros::NodeHandle n_;
        GTSAM::GTSAM_CORE Core_;
        ros::Subscriber imu_sub_;
        ros::Subscriber gps_sub_;
        ros::Subscriber linkstates_sub_;
        ros::Subscriber landmarks_sub_;
        std::thread isam2_thread_;
        std::thread output_thread_;
        std::thread rviz_thread_;
        Queue<std::shared_ptr<Measurement>, std::vector<std::shared_ptr<Measurement>>, MeasurementCompare> m_queue_;

        std::string imu_frame_id_;
        std::string gps_frame_id_;
        std::string map_frame_id_;
        std::string base_frame_id_;
        bool publish_visualization_markers_;
        ros::Publisher visualization_pub_;
        bool enable_landmarks_;
        tf::StampedTransform camera_to_imu_transform_;
        bool enable_linkstates;

        nav_msgs::Path path;

        NoiseParams init_params_;
        gtsam::Vector3 initial_linkstate_;
        Vector4 cur_baselink_orientation_;
        gtsam::Matrix33 initial_R_;
        tf::StampedTransform base_to_gps_transform_;
        gtsam::Vector3 initial_lla_;
        gtsam::Vector3 initial_ecef_;
        double initial_yaw_;
        gtsam::Vector3 Og0_to_Ob0_;
        
        std::ofstream file;
        std::string gps_file_path_;
        bool output_gps_;

        void subscribe();
        void mainIsam2Thread();
        void outputPublishingThread();
        void rvizPublishingThread();
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg); 
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void linkstatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);  
        //void landmarkCallback(const inekf_msgs::LandmarkArray::ConstPtr& msg);

        gtsam::Vector3 lla_to_ecef(const gtsam::Vector3& lla);
        gtsam::Vector3 lla_to_enu(const gtsam::Vector3& lla);

        //void publishLandmarkMeasurementMarkers(std::shared_ptr<LandmarkMeasurement> ptr);
};

#endif 
