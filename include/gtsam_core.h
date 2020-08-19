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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <chrono>
#include <string>
#include <boost/lockfree/queue.hpp>
#include <iostream>
#include <fstream>
#include <vector>
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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace GTSAM {

class GTSAM_CORE {
    public:


    private:


};

}   // end of namespace GTSAM

#endif 
