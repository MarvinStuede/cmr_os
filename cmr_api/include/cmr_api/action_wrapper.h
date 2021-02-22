/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   action_wrapper.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   07/2019
*
* @brief  Wrapper class to interface a simple action client, provides standard functions
*/
#pragma once

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmr_os/cmr_holder.h>
#include <iostream>
#include <mutex>
#include <actionlib_msgs/GoalStatusArray.h>
using namespace std;

template <typename TClient, typename TGoal, typename TFeedback, typename TResult>
class ActionWrapper
{
  typedef actionlib::SimpleActionClient<TClient> ActionClient;
public:
  ActionWrapper(){};
  ~ActionWrapper(){delete action_client_;}

  //! \brief Initialize the action client
  //!
  //! \param [in] topic for the client
  //! \param [in] async, if false the methods waits for the server
  void init(const std::string & topic, bool async=false){
    lock_guard<mutex> lg(mut_client_);
    if(!async) ROS_INFO_STREAM("Action client of type" << typeid(TClient).name() << " waiting for server");
    action_client_ = new ActionClient(topic, true);
    status_holder_.init(topic+"/status");
    if(!async){
      action_client_->waitForServer();
      ROS_INFO_STREAM("Action client of type" << typeid(TClient).name() << " connected to server!");
    }
    initialized_ = true;
  }
  //! \brief Check of action server is currently busy. By default, only checks if
  //! \param [in] check_server Check if there is an active goal on the server
  //! \return true if busy
  bool isBusy(bool check_server = false){
    lock_guard<mutex> lg(mut_client_);
    if(check_server) return is_busy_ || !serverNotBusy(status_holder_.get());
    else return is_busy_;
  }

  //! \brief Getter for action client
  //!
  //! \return Pointer to action client
  TClient* getClient(){
    lock_guard<mutex> lg(mut_client_);
    return action_client_;
  }

  //! \brief Wait for the current action
  //!
  //! \param [in] timeout, max time to wait
  //! \return false if timeout was reached
  bool waitForCurrentAction(double timeout = 0){
    lock_guard<mutex> lg(mut_client_);
    if(!is_busy_) return true;
    return action_client_->waitForResult(ros::Duration(timeout));
  }

  //! \brief Abort the current action
  //!
  void abortCurrentAction(){
    lock_guard<mutex> lg(mut_client_);
    action_client_->cancelAllGoals();
  }

  //! \brief Check if action was successful
  //!
  //! \return true if action succedes
  bool actionSucceeded(){
    lock_guard<mutex> lg(mut_client_);
    return action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  //! \brief Send goal to the server
  //! Sends a new goal to the server, only if it is not busy and already initialized.
  //!
  //! \param [in] goal Goal to send
  //! \return true if goal was sent
  bool sendGoal(const TGoal& goal){
    lock_guard<mutex> lg(mut_client_);
    if(is_busy_) return false;
    if(!initialized_) throw ros::Exception("Action Client not initialized");
    action_client_->sendGoal(goal,boost::bind(&ActionWrapper::onActionDone,this,_1,_2));
    is_busy_ = true;
    return true;
  }

  //! \brief Get the last received result
  //!
  //! \param [in] result reference
  void getResult(TResult& result){
    lock_guard<mutex> lg(mut_client_);
    result_holder_.get_value(result);
  }

  //! \brief Check if server is ready
  //!
  bool isReady(){
    lock_guard<mutex> lg(mut_client_);
    return initialized_ && action_client_->isServerConnected();
  }


protected:
  actionlib::SimpleActionClient<TClient>* action_client_;
  mutex mut_client_;

private:
  bool is_busy_ = false;
  bool initialized_ = false;
  cmr_os::cmrHolder<TResult> result_holder_;
  cmr_os::cmrStateHolder<actionlib_msgs::GoalStatusArrayPtr> status_holder_;

  /**
   * @brief Callback after action was completed. Saves the result to a holder object.
   * @param state
   * @param result
   */
  void onActionDone(const actionlib::SimpleClientGoalState& state, const TResult& result){
    lock_guard<mutex> lg(mut_client_);
    is_busy_ = false;
    result_holder_.set_value(result);
  }

  /**
   * @brief serverNotBusy Checks if the action server is currently executing
   * @param goal_ptr msg to check status of
   * @return  true if no goal is active
   */
  bool serverNotBusy(actionlib_msgs::GoalStatusArrayPtr goal_ptr){
    return std::all_of(goal_ptr->status_list.begin(), goal_ptr->status_list.end(),
                       [](actionlib_msgs::GoalStatus st){return st.status != st.ACTIVE;});
  }

};
