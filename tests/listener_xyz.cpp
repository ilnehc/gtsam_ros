#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>


using namespace std;

string filepath_odo = "/home/chenli/vrx_ws/src/vrx/gtsam_ros/tests/result/data_base_xyz_vrx.csv";
string filepath_rawGps = "/home/chenli/vrx_ws/src/vrx/gtsam_ros/tests/result/gps_297s.csv";
string filepath_rawImu = "/home/chenli/vrx_ws/src/vrx/gtsam_ros/tests/result/imu_297s.csv";
string filepath_filtered = "/home/chenli/vrx_ws/src/vrx/gtsam_ros/tests/result/data_filtered_xyz_vrx.csv";
ofstream file;

void rawCallback_base(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	int n = (msg->pose).size();
	double x = msg->pose[n-8].position.x;	// 8 is gps_link, 9 is base_link
	double y = msg->pose[n-8].position.y;
	double z = msg->pose[n-8].position.z;
	file.open(filepath_odo.c_str(), ios::app);
	file.precision(16);
	file << 0 << "," << x << "," << y << "," << z << endl;
	file.close();
}

void rawCallback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->latitude;
    double y = msg->longitude; 
    double z = msg->altitude;
	file.open(filepath_rawGps.c_str(), ios::app);
	file.precision(16);
	file << t1<< "," << x << "," << y << "," << z << endl;
	file.close();
}

void rawCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double ori_x = msg->orientation.x;
	double ori_y = msg->orientation.y;
	double ori_z = msg->orientation.z;
	double ori_w = msg->orientation.w;
	double ang_x = msg->angular_velocity.x;
	double ang_y = msg->angular_velocity.y;
	double ang_z = msg->angular_velocity.z;
	double lin_x = msg->linear_acceleration.x;
	double lin_y = msg->linear_acceleration.y;
	double lin_z = msg->linear_acceleration.z;
	file.open(filepath_rawImu.c_str(), ios::app);
	file.precision(16);
	file << t1 << "," << ori_x << "," << ori_y << "," << ori_z << "," << ori_w << "," << 
			ang_x << "," << ang_y <<  "," << ang_z << "," << 
			lin_x << "," << lin_y <<  "," << lin_z << endl;
	file.close();
}

void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("odometry: [%f]", msg->pose.pose.position.x);
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	double z = msg->pose.pose.position.z;
	file.open(filepath_odo.c_str(), ios::app);
	file << t1 << "," << x << "," << y << "," << z << endl;
	file.close();
}

void filterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  //ROS_INFO("filtered: [%f]", msg->pose.pose.position.x);
	double t = msg->header.stamp.toSec();
	size_t t1 = t * 1e9;
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;
	double z = msg->pose.pose.position.z;
	file.open(filepath_filtered.c_str(), ios::app);
	file.precision(16);
	file << t1 << "," << x << "," << y << "," << z << endl;
	file.close();
}

int main(int argc, char **argv)
{
	
	file.open(filepath_odo.c_str());	// base link
	file << "timestamp [ns]" << "," << "base x" << "," << "base y" << "," << "base z" << endl;
	file.close();/*
	file.open(filepath_rawGps.c_str());
	file << "timestamp [ns]" << "," << "latitude" << "," << "longitude" << "," << "altitude" << endl;
	file.close();
	file.open(filepath_rawImu.c_str());	// raw IMU data
	file << "timestamp [ns]" << "," << "orientation x" << "," << "orientation y" << "," << "orientation z" <<  "," << 
			"orientation w" <<  "," << "angular vel x" <<  "," << "angular vel y" <<  "," << "angular vel z" <<  "," << 
			"linear acc x" <<  "," << "linear acc y" <<  "," << "linear acc z" << endl;
	file.close();
	*/

	file.open(filepath_filtered.c_str());
	file << "timestamp [ns]" << "," << "filtered x" << "," << "filtered y" << "," << "filtered z" << endl;
	file.close();

  ros::init(argc, argv, "listener_xyz_gtsam_node");
  ros::NodeHandle n_;

  ros::Subscriber odo_sub_ = n_.subscribe("/gazebo/link_states", 1000, rawCallback_base);
  //ros::Subscriber imu_sub_ = n_.subscribe("/sensors/an_device/Imu", 1000, rawCallback_imu);
  //ros::Subscriber gps_sub_ = n_.subscribe("/sensors/an_device/NavSatFix", 1000, rawCallback_gps);
  ros::Subscriber filtered_sub_ = n_.subscribe("/wamv/robot_localization/odometry/filtered", 1000, filterCallback);

	ros::Rate loop_rate(500);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}