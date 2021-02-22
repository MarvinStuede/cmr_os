/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   ear_api.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   07/2019
*
* @brief  API Class for high level access of the Ear Movement
*/
#pragma once

#include "ros/ros.h"
#include <cmr_api/action_wrapper.h>
#include <std_msgs/Float64.h>
#include <cmr_msgs/LimbNum.h>
#include <sensor_msgs/JointState.h>
#include <cmr_msgs/LimbNum.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <boost/function.hpp>
#include <cmr_api/movement/moveable.h>


namespace cmr_api {

static constexpr double MAX_ANGLE_WIGGLE = 60.;
static constexpr double MAX_ANGLE_HANG = 110.;

/**
 * @brief The EarAPI class to access the EarAPI
 */
class EarAPI : public Moveable{


private :
  /**
   * @brief The CtrlManagement struct
   * Stores ROS specific settings
   */
  CtrlManagement params_left_, params_right_;

  ros::Subscriber sub_joint_state_;
public:
  EarAPI(){};
  ~EarAPI();
  /**
   * @brief Initialize the interface (must be called before first use)
   *
   * @param nh node handle to use
   */
  void init(ros::NodeHandle &nh);

  /**
   * @brief getJointVal
   * @param limbnum number of the limb (see cmr_msgs::LimbNum enum)
   * @return Joint value in rad
   */

  /**
   * @brief motionWiggle execute a wiggling motion
   * @param repeats number of repetitions
   * @param limbnum to use
   * @param max_angle_wiggle maximum angle to wiggle between
   * @param time_per_repeat to for each repetition
   * @return true if motion goal sent successfully
   */
  bool motionWiggle(double duration, int limbnum, double max_angle_wiggle = MAX_ANGLE_WIGGLE, double time_per_repeat = 0.4, double time_to_stay = 0.001);

  /**
   * @brief motionHang execute a hanging motion. Moves the ears back and stays there for a specific time
   * @param dur_movement duration to let the ears hang
   * @param limbnum to use
   * @param max_angle_hang angle to move to
   * @return true if motion goal sent successfully
   */
  bool motionHang(double dur_total, double dur_movement,int limbnum, double max_angle_hang = MAX_ANGLE_HANG);

  bool motionGoTo(double dur_movement,int limbnum, double max_angle_hang = MAX_ANGLE_HANG);


private:


  ros::NodeHandle* nh_;
  ros::ServiceClient serv_switch_controller_;

};

}
