/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   move_base_client.h
 * @author Simon Ehlers
 * @date   09/2019
*
* @brief A client and standard functions for a move base client
*/


#pragma once
#include <cmr_api/action_wrapper.h>
#include <move_base/move_base.h>

namespace cmr_api{
using MoveBaseAction = move_base_msgs::MoveBaseAction;
using MoveBaseGoal = move_base_msgs::MoveBaseGoal;
using MoveBaseFeedback = move_base_msgs::MoveBaseFeedback;
using MoveBaseResult = move_base_msgs::MoveBaseResultConstPtr;
class MoveBaseClient :
    public ActionWrapper<MoveBaseAction,MoveBaseGoal,MoveBaseFeedback,MoveBaseResult>{

public:
  MoveBaseClient(){;};

private:
  bool is_active_ = false;

};
}
