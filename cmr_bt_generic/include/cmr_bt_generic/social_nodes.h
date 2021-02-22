/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   social_nodes.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2020
*
* @brief  Behavior Tree Nodes for social functions (speech, LED expressions...)
*/
#pragma once

#include "ros/ros.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <cmr_api/speech_api.h>
#include <cmr_api/led_api.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "cmr_msgs/DemoPause.h"


namespace cmr_bt {

class Speak : public BT::CoroActionNode
{
public:

  Speak(const std::string& name, const BT::NodeConfiguration& config):
    BT::CoroActionNode(name, config)
  {
    speech_.init(_node, true);
  }

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("text")
    };
  }

private:

  BT::NodeStatus tick() override
  {
    if(!speech_.getTTSClient()->isReady()) return BT::NodeStatus::FAILURE;

    if(!_text_sent){
      auto text = getInput<std::string>("text");
      if(!text)  throw BT::RuntimeError("error reading port [target]:", text.error());

      speech_.getTTSClient()->say(text.value(), false);
       _text_sent = true;
    }

    if(speech_.getTTSClient()->isBusy()) setStatusRunningAndYield();

    else{
      if(speech_.getTTSClient()->actionSucceeded()){
        _text_sent = false;
        return BT::NodeStatus::SUCCESS;
      }
      else{
        _text_sent = false;
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::RUNNING;
  }
  void halt() override
  {
    _aborted = true;
    speech_.getTTSClient()->abortCurrentAction();
    CoroActionNode::halt();
  }

  bool _aborted = false;
  bool _text_sent = false;
  ros::NodeHandle _node;
  cmr_api::Speech speech_;
};


class FacialExpression : public BT::SyncActionNode
{
public:

  FacialExpression(const std::string& name, const BT::NodeConfiguration& config):
    BT::SyncActionNode(name, config)
  {
    _node = std::make_unique<ros::NodeHandle>();
    _led = std::make_unique<cmr_api::LEDAPI>();
    _led->init(*_node);
  }

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("expression"),
      BT::InputPort<double>("duration")
    };
  }

private:

  BT::NodeStatus tick() override{
      auto exp = getInput<std::string>("expression");
      if(!exp.has_value())  throw BT::RuntimeError("error reading port [exp]:", exp.error());
      auto dur = getInput<double>("duration");
      if(!dur.has_value())  throw BT::RuntimeError("error reading port [dur]:", dur.error());
      if(_led->getPanel()->setExpression(cmr_api::stringToExpression(exp.value()), dur.value())) return BT::NodeStatus::SUCCESS;
      else return BT::NodeStatus::FAILURE;
  }

  std::unique_ptr<ros::NodeHandle> _node;
  std::unique_ptr<cmr_api::LEDAPI> _led;
};
/**
 * @brief The InteractionWithTablet class
 * Based on a message from the tablet, checks whether someone is interacting with the robot (using the tablet)
 * Tablet sends a cmr_msg::DemoPause on every tablet interaction
 */
class InteractionWithTablet : public BT::ConditionNode
{

public:
  InteractionWithTablet(const std::string& name)
    : InteractionWithTablet::ConditionNode(name, {})
  {
    _node = std::make_unique<ros::NodeHandle>();
    _sub_interaction = _node->subscribe("/sobi/speech/demo_pause",10, &InteractionWithTablet::subInteractionCb,this);
  }

  BT::NodeStatus tick() override{
    if(ros::Time::now() < _time_blocked){
      return BT::NodeStatus::SUCCESS;
    }
    else {
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  ros::Subscriber _sub_interaction;
  std::unique_ptr<ros::NodeHandle> _node;
  ros::Time _time_blocked = ros::Time(0);

  void subInteractionCb(const cmr_msgs::DemoPause &msg){
    //If signal received, determine the time until when interaction is happening
    if(msg.pause){
      _time_blocked = ros::Time::now() + ros::Duration(msg.duration);
    }
  }

};

inline void registerSocialNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<FacialExpression>("FacialExpression");
  factory.registerNodeType<Speak>("Speak");
  factory.registerNodeType<InteractionWithTablet>("InteractionWithTablet");
}

}
