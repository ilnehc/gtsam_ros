/* ----------------------------------------------------------------------------
 * Copyright 2020, Li Chen <ilnehc@umich.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gtsam_node.cpp
 *  @author Li Chen
 *  @brief  Creates ROS node that runs the GTSAM
 *  @date   August 19, 2020
 **/

#include "ros/ros.h"
#include "gtsam_ros.h"
#include <chrono>

using namespace std;

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "gtsam_node");
    ros::NodeHandle n;    

    // Initialize GTSAM ROS wrapper
    GTSAM_ROS gtsam_wrapper(n);
    gtsam_wrapper.init();
    gtsam_wrapper.run();

    return 0;
}
