/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   call_supervisor_client.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
*
* @brief A client and standard functions for a call supervisor client
*/

#pragma once
#include <cmr_api/action_wrapper.h>
#include <cmr_msgs/CallSupervisorAction.h>


namespace cmr_api{
using CallSupervisorAction = cmr_msgs::CallSupervisorAction;
using CallSupervisorGoal = cmr_msgs::CallSupervisorGoal;
using CallSupervisorFeedback = cmr_msgs::CallSupervisorFeedback;
using CallSupervisorResult = cmr_msgs::CallSupervisorResultConstPtr;
class CallSupervisorClient :
    public ActionWrapper<CallSupervisorAction,CallSupervisorGoal,CallSupervisorFeedback,CallSupervisorResult>{

public:
  /**
   * @brief CallSupervisorClient Constructor

   */
  CallSupervisorClient(bool async=false){
    init("/sobi/call_supervisor", async);
  }
  bool callSupervisor(const std::string &msg)
  {
    CallSupervisorGoal goal;
    goal.message = msg;

    if(sendGoal(goal)) return true;
    else{
      ROS_ERROR("Call supervisor server not active, not sending goal!");
      return false;
    }
  }

private:


};
}
