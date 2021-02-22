#include <cmr_bt_generic/system_nodes.h>

BT_REGISTER_NODES(factory)
{
  cmr_bt::registerSystemNodes(factory);
}

BT::NodeStatus cmr_bt::SendMessage::tick()
{

  std::string service = cmr_bt::checkInput(getInput<std::string>("service"));
  cmr_msgs::SendMessage srv;
  //Get service from Message consts
  if(service == "EMAIL") srv.request.service = srv.request.EMAIL;
  else if(service == "TELEGRAM") srv.request.service = srv.request.TELEGRAM;
  else{
    ROS_ERROR_STREAM("Service '"+service+"' not found. No message sent.");
    return BT::NodeStatus::FAILURE;
  }
  //Get message from blackboard
  srv.request.message = cmr_bt::checkInput(getInput<std::string>("message"));

  _srv_client.call(srv);

  if(srv.response.success) return BT::NodeStatus::SUCCESS;
  else return BT::NodeStatus::FAILURE;

}

BT::NodeStatus cmr_bt::CallSupervisor::tick()
{
  //Helper lambdas to reset init and return result
  auto failure = [&]{_init = false; return BT::NodeStatus::FAILURE;};
  auto success = [&]{_init = false; return BT::NodeStatus::SUCCESS;};

  if(!_init){
    //Get message from blackboard
    std::string message = cmr_bt::checkInput(getInput<std::string>("message"));
    //Call the server. If not connection established, fail immediately
    if(!_client->callSupervisor(message)) return failure();
    _init = true;
  }
  //As long as action is active (Supervisor did not respond yet), set status running
  if(_client->isBusy()) setStatusRunningAndYield();

  else{
    if(_client->actionSucceeded()){ //If supervisor responded
      cmr_api::CallSupervisorResult result;
      _client->getResult(result);
      if(result->fixed) //If problem was marked as fixed
        return success();
      else
        return failure();
    }
    else{
      return failure();
    }
  }
  return BT::NodeStatus::RUNNING;

}
