/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   arm_api.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   07/2019
*
* @brief  API Class for high level access of the Arm Movement
*/
#pragma once

#include "ros/ros.h"
#include <cmr_api/action_wrapper.h>
#include <std_msgs/Float64.h>
#include <cmr_msgs/LimbNum.h>
#include <sensor_msgs/JointState.h>
#include <cmr_msgs/LimbNum.h>
#include <cmr_os/cmr_holder.h>
#include <cmr_api/movement/moveable.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <boost/function.hpp>
#include <memory>

using namespace std;
namespace cmr_api {

/**
 * @brief The ArmAPI class to access the ArmAPI
 */
class ArmAPI : public Moveable{

public:
  ArmAPI();
  ~ArmAPI();

  /**
    * @brief Initialize the object
    * @param nh Node handle to use
    */
   void init(ros::NodeHandle &nh);

   /**
    * @brief Move to a specific angular position and stay there for a duration before moving to 0
    * @param angle to move to
    * @param dur_stay duration to stay
    * @param limbnum to use
    * @return  true if movement goal sent
    */
   bool moveToPosAndStay(double angle, double dur_stay,int limbnum);

   /**
    * @brief move to a specific angular position
    * @param angle angular position to move to
    * @param limbnum limbnum to usw
    * @return true if movement goal sent
    */
   bool moveToPos(double angle, int limbnum);

private:
  ros::NodeHandle* nh_;

   CtrlManagement params_left_, params_right_;



};

}
