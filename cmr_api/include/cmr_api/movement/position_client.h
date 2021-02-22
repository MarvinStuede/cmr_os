/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   position_client.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   08/2019
*
* @brief A client and standard functions for a joint position client
*/

#pragma once
#include <cmr_api/action_wrapper.h>
#include <cmr_api/movement/ctrl_management.h>
#include <controller_manager_msgs/SwitchController.h>
#include <std_msgs/Float64.h>
#include <thread>
/**
 * @brief The PositionClient class
 * Accesses a JointPositionController
 */
namespace cmr_api{
class PositionClient{

public:
    /**
   * @brief PositionClient Constructor
   * @param ctrl_management struct with ros settings
   */
    PositionClient(CtrlManagement &ctrl_management){
        ctrl_management_ = &ctrl_management;
        pub_angle_ = ctrl_management_->nh->advertise<std_msgs::Float64>("/sobi/"+ctrl_management_->pos_ctrl_name+"/command",100);
    }
    /**
   * @brief Go to a specific angular position
   * @param angle in rad
   * @return true if angle was sent to action server
   */
    bool goToPos(double angle)
    {
        if(!is_active_){
            ROS_ERROR("Position controller not active, not sending goal angle!");
            return false;
        }
        std_msgs::Float64 angle_msg;
        angle_msg.data = angle;
        pub_angle_.publish(angle_msg);
    }
    /**
     * @brief moveToPosAndStay Moves to a position and stays there for a defined time. Then moves to 0.
     * @param angle position to move to
     * @param dur_stay duration to stay at position
     * @return true if goal send successfully
     */
    bool moveToPosAndStay(double angle, double dur_stay = 2.5){
        if(!activate()) return false;

        //TODO: Make nice, get rid of undefined behavior
        //Thread gets detached...
        auto toZero = [=](double dur){
            ros::Duration(dur_stay).sleep();
            goToPos(0.);
        };
        auto thread = std::thread(toZero,dur_stay);
        thread.detach();
        return goToPos(angle);
    }
    /**
   * @brief Activate this client
   * @return true if successful
   */
    bool activate()
    {
        if(is_active_) return true;
        is_active_ = ctrl_management_->switchController(ctrl_management_->traj_ctrl_name,ctrl_management_->pos_ctrl_name);
        return is_active_;

    }
private:
    bool is_active_ = false;
    CtrlManagement* ctrl_management_;
    ros::Publisher pub_angle_;

};
}
