/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   shutdown_node.cpp
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   2021
*
* @brief  Node that does nothing. Can be used as a "required" node in a launch file to quit the launch file if node is killed
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shutdown_node");

    ros::NodeHandle node_handle("~");

    ros::spin();

    return 0;
}

