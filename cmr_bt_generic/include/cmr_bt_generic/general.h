/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   general.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
*
* @brief  General functions and nodes
*/
#pragma once

#include "ros/ros.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <sensor_msgs/BatteryState.h>
#include "behaviortree_cpp_v3/bt_factory.h"

namespace cmr_bt {

/**
 * check if Blackboard entry exists and return
 */
template <typename T>
T checkInput(const BT::Optional<T> &opt)
{
  if(!opt.has_value()){
    throw BT::RuntimeError("error reading port:", opt.error());
  }
  return opt.value();
}
/**
 * @brief The ROSOut class
 * Write a blackbaord entry to rosout
 */
class ROSOut : public BT::SyncActionNode
{
public:
  ROSOut(const std::string& name, const BT::NodeConfiguration& config):
    BT::SyncActionNode(name, config){
    _node = std::make_unique<ros::NodeHandle>();
  }
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("msg"),
          BT::InputPort<std::string>("level"),
          BT::InputPort<int>("throttle_period"),
    };
  }

private:
  BT::NodeStatus tick() override;
  std::unique_ptr<ros::NodeHandle> _node;
};
/**
 * @brief The SaveTimestamp class
 * Save a ROS timestamp to the blackboard
 */
class SaveTimestamp : public BT::SyncActionNode
{
public:
  SaveTimestamp(const std::string& name, const BT::NodeConfiguration& config):
    BT::SyncActionNode(name, config){
  }
  static BT::PortsList providedPorts(){
    return {
      BT::OutputPort<double>("timestamp")
    };
  }

private:
  BT::NodeStatus tick() override{
    setOutput("timestamp",ros::Time::now().toSec());
    return BT::NodeStatus::SUCCESS;
  };
};
/**
 * @brief The TimePassed class
 * Condition to check if a duration has passed since a given timestamp
 */
class TimePassed : public BT::ConditionNode
{

public:
  TimePassed(const std::string& name, const BT::NodeConfiguration& config)
    : TimePassed::ConditionNode(name, config)
  {

  }
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<double>("timestamp"),
          BT::InputPort<double>("duration")
    };
  }
  BT::NodeStatus tick() override{
    double timestamp = cmr_bt::checkInput(getInput<double>("timestamp"));
    double duration = cmr_bt::checkInput(getInput<double>("duration"));
    // Create time object from timestamp
    ros::Time time;
    time.fromSec(timestamp);
    bool passed = ros::Time::now() > (time + ros::Duration(duration));

    return passed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  };
};

/**
 * @brief The ROSParamToBlackboard class
 * Write a ros paramter to blackboard
 */
class ROSParamToBlackboard : public BT::SyncActionNode
{
public:
  ROSParamToBlackboard(const std::string& name, const BT::NodeConfiguration& config):
    BT::SyncActionNode(name, config){
    _node = std::make_unique<ros::NodeHandle>("~");
  }
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("param"),
          BT::OutputPort<std::string>("entry")
    };
  }

private:
  BT::NodeStatus tick() override;
  std::unique_ptr<ros::NodeHandle> _node;
};

inline void registerGeneralNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<SaveTimestamp>("SaveTimestamp");
  factory.registerNodeType<TimePassed>("TimePassed");
  factory.registerNodeType<ROSParamToBlackboard>("ROSParamToBlackboard");
  factory.registerNodeType<ROSOut>("ROSOut");

}
};
