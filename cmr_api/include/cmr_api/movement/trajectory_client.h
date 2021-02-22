/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   trajectory_client.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief A client and standard functions for a joint trajectory client
*/

#pragma once
#include <cmr_api/action_wrapper.h>
#include <cmr_api/movement/ctrl_management.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


namespace cmr_api{
using JointTrajAction = control_msgs::FollowJointTrajectoryAction;
using JointTrajGoal = control_msgs::FollowJointTrajectoryGoal;
using JointTrajFeedback = control_msgs::FollowJointTrajectoryFeedback;
using JointTrajResult = control_msgs::FollowJointTrajectoryResultConstPtr;
class TrajectoryClient :
    public ActionWrapper<JointTrajAction,JointTrajGoal,JointTrajFeedback,JointTrajResult>{

public:
  /**
   * @brief TrajectoryClient Constructor
   * @param ctrl_management struct with ros settings
   */
  TrajectoryClient(CtrlManagement &ctrl_management){
    ctrl_management_ = &ctrl_management;
    init("/sobi/"+ctrl_management_->traj_ctrl_name+"/follow_joint_trajectory");
  }

  /**
   * @brief Drive a trajectory
   * @param traj trajectory to drive
   * @return true of trajectory was sent to action server
   */
  bool goTrajectory(const trajectory_msgs::JointTrajectory &traj)
  {
    JointTrajGoal goal;
    goal.trajectory = traj;

    //goal.header.stamp = ros::Time::now();
    if(sendGoal(goal)) return true;
    else{
      ROS_ERROR("Ear Trajectory controller not active, not sending goal trajectory!");
      return false;
    }
  }
  /**
   * @brief Activate this client
   * @return true if successful
   */
  bool activate()
  {
    lock_guard<mutex> lg(mut_client_);
    if(is_active_) return true;
    if(!action_client_->isServerConnected()) return false;

    is_active_ = ctrl_management_->switchController(ctrl_management_->pos_ctrl_name,ctrl_management_->traj_ctrl_name);
    return is_active_;

  }

  /**
   * @brief motionWiggle Does a wiggling motion
   * Creates a trajectory between \p max_angle and - \p max_angle
   * @param repeats number of repetitions
   * @param max_angle Maximum angle to move between
   * @param time_per_repeat time for each period
   * @return true if trajectory successfully send as goal
   */
  bool motionWiggle(uint32_t repeats, double max_angle, double time_per_repeat = 0.4, double time_to_stay = 0.001)
  {

    if(!activate()) return false;
    if(repeats == 0) return false;
    unique_lock<mutex> ul(mut_client_);

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back(ctrl_management_->joint_name);

    //Resulting Trajectory point depend on the number of repeats (back and forth)
    uint32_t traj_points = repeats * 4 + 1;
    trajectory.points.resize(traj_points);

    //Define a signum to move between positive max_angle and negative_max angle
    int sgn = 1;
    for(uint32_t i = 0, k = 0; i < traj_points; i = i + 2, k++){

      //Set time for point
      trajectory.points[i].time_from_start = ros::Duration((k+1)* time_per_repeat +  k * time_to_stay);
      trajectory.points[i].positions.resize(1);
      trajectory.points[i].velocities.resize(1);

      //Last point: move to 0.0
      if(i == traj_points - 1){
        trajectory.points[i].positions[0] = 0.0;
        trajectory.points[i].velocities[0] = 0.0;
      }
      else{
        //Stay for specified time
        trajectory.points[i+1].time_from_start = ros::Duration((k+1)* time_per_repeat + (k+1) * time_to_stay);
        trajectory.points[i+1].positions.resize(1);
        trajectory.points[i+1].velocities.resize(1);

        trajectory.points[i].positions[0] = deg2rad(sgn*max_angle);
        trajectory.points[i].velocities[0] = 0.0;
        trajectory.points[i+1].positions[0] = deg2rad(sgn*max_angle);
        trajectory.points[i+1].velocities[0] = 0.0;
        sgn*=-1;
      }
    }
    ul.unlock();
    return goTrajectory(trajectory);
  }

  /**
   * @brief moveToPosAndStay Moves to a specific position and stays for specific duration
   * @param max_angle Angle to move to
   * @param dur_stay duration to stay at angle
   * @param dur_move duration for movement to angle
   * @return true if trajectory successfully send as goal
   */
  bool moveToPosAndStay(double max_angle, double dur_stay = 2.0, double dur_move = 1.0)
  {

    if(!activate()) return false;

    if(dur_stay < 2 * dur_move){
      ROS_ERROR("Trajectory Controller: moveToPosAndStay time for movement too small");
      return false;
    }

    unique_lock<mutex> ul(mut_client_);
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back(ctrl_management_->joint_name);

    //Resulting Trajectory point depend on the number of repeats (back and forth)
    uint32_t traj_points = 3;
    trajectory.points.resize(traj_points);

    for(uint32_t i = 0;i<traj_points;i++){
      trajectory.points[i].positions.resize(1);
      trajectory.points[i].velocities.resize(1);
      trajectory.points[i].velocities[0] = 0.0;
    }

    //Move to Max angle in specified time
    trajectory.points[0].time_from_start = ros::Duration(dur_move);
    trajectory.points[0].positions[0] = deg2rad(max_angle);

    //Stay at max angle for specified time
    trajectory.points[1].time_from_start = ros::Duration(dur_stay - dur_move);
    trajectory.points[1].positions[0] = deg2rad(max_angle);

    //Move back to zero in specified time
    trajectory.points[2].time_from_start = ros::Duration(dur_stay);
    trajectory.points[2].positions[0] = 0.0;


    ul.unlock();
    return goTrajectory(trajectory);
  }

  bool moveToPos(double max_angle, double dur_move = 1.0)
  {
    if(!activate()) return false;

    unique_lock<mutex> ul(mut_client_);
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back(ctrl_management_->joint_name);

    trajectory.points.resize(1);
    trajectory.points[0].positions.resize(1);
    trajectory.points[0].velocities.resize(1);

    //Move to Max angle in specified time
    trajectory.points[0].velocities[0] = 0.0;
    trajectory.points[0].time_from_start = ros::Duration(dur_move);
    trajectory.points[0].positions[0] = deg2rad(max_angle);

    ul.unlock();
    return goTrajectory(trajectory);
  }


  /**
   * @brief Creates a trajectory to a defined angle
   * @param traj JointTrajectory to write
   * @param angle_deg end angle
   * @param time time for movement
   */
  void createTrajToAngle(trajectory_msgs::JointTrajectory &traj, double angle_deg, double time){
    lock_guard<mutex> lg(mut_client_);
    traj.joint_names.push_back(ctrl_management_->joint_name);
    traj.points.resize(1);
    traj.points[0].positions.resize(1);
    traj.points[0].velocities.resize(1);
    traj.points[0].positions[0] = deg2rad(angle_deg);
    traj.points[0].velocities[0] = 0.0;
    traj.points[0].time_from_start = ros::Duration(time);
    return;
  }
private:
  bool is_active_ = false;
  CtrlManagement* ctrl_management_;

};
}
