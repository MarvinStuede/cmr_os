/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   neutral_mover.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief Neutral mover sends random angles to a joint controller in a specific interval
*/
#pragma once

#include <random>
#include "ros/ros.h"
#include <cmr_os/cmr_holder.h>
#include <iostream>
#include <mutex>
#include <std_msgs/Float64.h>
#include <cmr_msgs/LimbNum.h>
#include <sensor_msgs/JointState.h>
#include <cmr_msgs/LimbNum.h>
#include <cmr_api/movement/position_client.h>
#include <cmr_api/movement/trajectory_client.h>
#include <thread>
#include <atomic>
#include <boost/function.hpp>

static constexpr double MAX_ANGLE_NEUTRAL = 20.;
static constexpr double INTERVAL_NEUTRAL = 4.;

namespace cmr_api {
/**
 * @brief Parameters for the neutral mover
 */
struct NMovParams{
    NMovParams(){;}
    double max_angle = 20.;
    double min_angle = -max_angle;
    //Interval for movement duration
    double min_time = 2.5;
    double max_time = 3.5;
};
class NeutralMover{
public:
    /**
     * @brief NeutralMover constructor
     * @param client Joint trajectory client
     * @param params Parameters to use
     */
    NeutralMover(shared_ptr<TrajectoryClient> client, NMovParams params = NMovParams()){
        traj_client = client;
        params_ = params;
    }
    /**
     * @brief NeutralMover constructor
     * @param client Joint position client
     * @param params Parameters to use
     */
    NeutralMover(shared_ptr<PositionClient> client, NMovParams params = NMovParams()){
        pos_client = client;
        params_ = params;
    }

    /**
     * @brief isExecuting
     * @return true if executing
     */
    bool isExecuting(){return executing;}

    /**
     * @brief stop neutral movement
     */
    void stop(){executing = false;}

    /**
     * @brief start neutral movement
     * @return true if started
     */
    bool start();



private:
    std::thread thread;
    std::atomic_bool executing;
    shared_ptr<TrajectoryClient> traj_client;
    shared_ptr<PositionClient> pos_client;
    ros::NodeHandle nh;
    ros::Timer timer;
    NMovParams params_;

    /**
     * @brief createMotionNeutral starts the timer
     */
    void createMotionNeutral();

    /**
     * @brief Timer callback to call repeatedly
     * Creates random values and sends them to the joint controller
     */
    void motionNeutral(const ros::TimerEvent&);

    /**
     * @brief getRandInInterval Get a random value in an interval
     * @param lower_bound
     * @param upper_bound
     * @return random value
     */
    double getRandInInterval(double lower_bound, double upper_bound);

};
}
