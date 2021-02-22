/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   system_nodes.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
*
* @brief  Behavior Tree Nodes for system related functions
*/
#pragma once

#include "ros/ros.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <sensor_msgs/BatteryState.h>
#include <neo_msgs/RelayBoardV2.h>
#include "std_msgs/Bool.h"
#include <cmr_msgs/SendMessage.h>
#include "cmr_bt_generic/general.h"
#include "cmr_api/call_supervisor_client.h"
#include <behaviortree_cpp_v3/bt_factory.h>

namespace cmr_bt {
/**
 * @brief The Charging class
 * Checks if the robot is currently charging
 */
class Charging : public BT::ConditionNode
{

public:
  Charging(const std::string& name)
    : Charging::ConditionNode(name, {})
  {
    node_ = std::make_unique<ros::NodeHandle>();
    sub_battery_charge_state_ = node_->subscribe("/base/relayboard_v2/charging",10, &Charging::subChargeStateCallback,this);
  }

  BT::NodeStatus tick() override{
    if(!charging){
      return BT::NodeStatus::FAILURE;
    }else{
      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  ros::Subscriber sub_battery_charge_state_;
  std::unique_ptr<ros::NodeHandle> node_;
  bool charging = false;
  void subChargeStateCallback(const std_msgs::Bool &msg){
    this->charging = msg.data;
  }

};
/**
 * @brief The SendMessage class
 * Send a message via email or telegram
 */
class SendMessage : public BT::SyncActionNode
{
public:

  SendMessage(const std::string& name, const BT::NodeConfiguration& config):
    BT::SyncActionNode(name, config)
  {
    _node = std::make_shared<ros::NodeHandle>("~");
    _srv_client = _node->serviceClient<cmr_msgs::SendMessage>("/sobi/send_message");

  }

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("message"),
      BT::InputPort<std::string>("service")
    };
  }

private:


  BT::NodeStatus tick() override;

  std::shared_ptr<ros::NodeHandle> _node;
  ros::ServiceClient _srv_client;

};
/**
 * @brief The CallSupervisor class
 * Send a message via telegram and wait until a supervisor responded to the message. Depending on the response, return SUCCESS or failure
 */
class CallSupervisor : public BT::CoroActionNode
{
public:

  CallSupervisor(const std::string& name, const BT::NodeConfiguration& config):
    BT::CoroActionNode(name, config)
  {
    _client = std::make_unique<cmr_api::CallSupervisorClient>(true);
  }

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("message")
    };
  }

private:

  BT::NodeStatus tick() override;

  void halt() override
  {
    _aborted = true;
    _client->abortCurrentAction();
    CoroActionNode::halt();
  }

  bool _aborted = false;
  bool _init = false;
  ros::NodeHandle _node;
  std::unique_ptr<cmr_api::CallSupervisorClient> _client;
};

inline void registerSystemNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<CallSupervisor>("CallSupervisor");
  factory.registerNodeType<SendMessage>("SendMessage");
  factory.registerNodeType<Charging>("Charging");
}

};
