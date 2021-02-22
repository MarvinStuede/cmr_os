/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   tablet_menu_handler.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   09/2019
*
* @brief  Handles User interaction on the tablet and does a speech output
*/
#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "cmr_api/speech_api.h"
#include "cmr_msgs/ShowFragment.h"
using namespace  std;
constexpr char node_name[] = "tablet_menu_handler";

/**
 * @brief The TabletHandler class
 */
class TabletHandler
{
public:
    /**
     * @brief TabletHandler
     * @param node_handle
     */
    TabletHandler(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    cmr_api::Speech speech_;

    ros::Timer timer_state_;

    cmr_os::cmrStateHolder<cmr_msgs::ShowFragment> hold_current_frag_;
    cmr_msgs::ShowFragment current_frag_;
    map<string,ros::Time> said_sentences_;
    double dur_sentence_lock_ = 60.;

    void timerCallback(const ros::TimerEvent &evt);
    /**
     * @brief say a text
     * @param str text to say
     */
    void say(const string &str);
    /**
     * @brief Check if strings are equal and not the same as current fragments name
     * @param str
     * @param comp_str
     * @return true if equal
     */
    bool equalToStrAndNew(const string& str, const string& comp_str){return str==comp_str && str != current_frag_.name;}
};
