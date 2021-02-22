#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>
#include <cmr_bt_generic/social_nodes.h>
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <functional>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"bt_tester");
  ros::NodeHandle node_handle;


  using namespace BT;

  //Creating an Instance to Create or Build your BT
  std::unique_ptr<BT::BehaviorTreeFactory> factory = std::make_unique<BT::BehaviorTreeFactory>();


  //register and assign Action Nodes from xml File
  factory->registerNodeType<Speak>("Speak");


  //Build Tree from File
  std::string ppath = ros::package::getPath("cmr_bt_generic");
  auto tree = factory->createTreeFromFile(ppath+"/xml/TestTree.xml");


  //if you want to see in the console which nodes are in which status then comment this in.
  BT::StdCoutLogger logger_cout(tree);

  //Define here Frequence in which BT gets ticked from root node
  ros::Rate r(10);


  while(ros::ok())
  {

    ros::spinOnce();

   tree.rootNode()->executeTick();
    r.sleep();
  }

  return 0;
}
