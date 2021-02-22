#include <cmr_bt_generic/general.h>
#include <ros/console.h>
BT_REGISTER_NODES(factory)
{
  cmr_bt::registerGeneralNodes(factory);
}

BT::NodeStatus cmr_bt::ROSOut::tick()
{
  std::string msg = checkInput(getInput<std::string>("msg"));
  std::string level = checkInput(getInput<std::string>("level"));
  unsigned int period = checkInput(getInput<unsigned int>("throttle_period"));

  if(level == "DEBUG"){
    ROS_DEBUG_THROTTLE(period, msg.c_str());
  }
  else if(level == "INFO"){
    ROS_INFO_THROTTLE(period, msg.c_str());
  }
  else if(level == "WARN"){
    ROS_WARN_THROTTLE(period, msg.c_str());
  }
  else if(level == "ERROR"){
    ROS_ERROR_THROTTLE(period, msg.c_str());
  }
  else if(level == "FATAL"){
    ROS_FATAL_THROTTLE(period, msg.c_str());
  }
  else {
    ROS_ERROR("ROSOut Node: Invalid level");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus cmr_bt::ROSParamToBlackboard::tick()
{
    std::string param = checkInput(getInput<std::string>("param"));
    if(!_node->hasParam(param)){
      ROS_ERROR_STREAM("Parameter "<<param<<" does not exist");
      return BT::NodeStatus::FAILURE;
    }
    std::string value;
    _node->getParam(param, value);
    setOutput("entry", value);
    return BT::NodeStatus::SUCCESS;
}
