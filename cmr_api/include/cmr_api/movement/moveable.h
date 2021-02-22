/* *****************************************************************
*
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   moveable.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief  Base class for movers, eg. arms or ears
*/
#pragma once

#include "ros/ros.h"
#include <cmr_os/cmr_holder.h>
#include <iostream>
#include <mutex>
#include <std_msgs/Float64.h>
#include <cmr_msgs/LimbNum.h>
#include <sensor_msgs/JointState.h>
#include <cmr_msgs/LimbNum.h>
#include <cmr_api/movement/neutral_mover.h>

using namespace std;
using JointState = std::map<std::string,double>;
namespace cmr_api{

/**
 * @brief The Moveable class, holds joint clients and standard functions
 */
class Moveable{
public:
    Moveable();
    /**
     * @brief Initialize
     * @param nh Node handle to use
     */
    virtual void init(ros::NodeHandle &nh) = 0;

    /**
     * @brief getJointVal get the current joint value
     * @param limbnum to receive
     * @return angle in rad
     */
    double getJointVal(int limbnum);

    /**
     * @brief startMotionNeutral starts neutral movement
     * Neutral movement moves to random joint positions in an interval (see neutral_mover.h)
     * @param limbnum to move
     * @return true if neutral movement started successfully
     */
    bool startMotionNeutral(int limbnum);

    /**
     * @brief stopMotionNeutral stops neutral movement
     * @param limbnum to stop
     * @param wait_time time to wait after neutral movement was stopped.
     * Can be used to ensure that the joint controller is ready again after function returns
     */
    void stopMotionNeutral(int limbnum, double wait_time = 0.1);

    /**
     * @brief neutralActivated check if neutral movement is currently active
     * @param limbnum to check
     * @return true if active
     */
    bool neutralActivated(int limbnum);

    /**
     * @brief abortMovements Abort current movement
     * @param limbnum to abort
     */
    void abortMovements(int limbnum);

    /**
       * @brief Getter for left position client
       * @return Pointer to left position client
       */
    shared_ptr<PositionClient> getPosClientLeft(){return poscl_left_;}

    /**
         * @brief Getter for right position client
         * @return Pointer to right position client
         */
    shared_ptr<PositionClient> getPosClientRight(){return poscl_right_;}

    /**
         * @brief Getter for left trajectory client
         * @return Pointer to left trajectory client
         */
    shared_ptr<TrajectoryClient> getTrajClientLeft(){return trajcl_left_;}

    /**
         * @brief Getter for right trajectory client
         * @return Pointer to right trajectory client
         */
    shared_ptr<TrajectoryClient> getTrajClientRight(){return trajcl_right_;}

protected:

    shared_ptr<PositionClient> poscl_left_, poscl_right_;
    shared_ptr<TrajectoryClient> trajcl_left_, trajcl_right_;
    shared_ptr<NeutralMover> nmov_left_, nmov_right_;

    set<shared_ptr<PositionClient>> getPosClientsFromLimbNum(int limbnum);
    set<shared_ptr<TrajectoryClient>> getTrajClientsFromLimbNum(int limbnum);
    set<shared_ptr<NeutralMover>> getNeutralMoversFromLimbNum(int limbnum);

private:
    cmr_os::cmrStateHolder<sensor_msgs::JointStatePtr> js_holder_;

    /**
     * @brief getJointNameFromLimbNum Get the joint name as a string
     * @param limbnum to get
     * @return name of joint
     */
    std::string getJointNameFromLimbNum(int limbnum);
};
}
