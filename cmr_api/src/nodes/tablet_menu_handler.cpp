/* *****************************************************************
 *
 * cmr_api
 *
 * Copyright (c) %YEAR%,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/**
 * @file   %FILENAME%
 * @author %USER% (%$EMAIL%)
 * @date   %DATE%
 *
 * @brief  Filedescription
 */

#include "cmr_api/tablet_menu_handler.h"
using namespace std;
//########## CONSTRUCTOR ###############################################################################################
TabletHandler::TabletHandler(ros::NodeHandle &node_handle):
  node_(&node_handle)
{

  speech_.init(*node_);

 hold_current_frag_.init("/current_fragment");
  timer_state_ = node_->createTimer(ros::Duration(0.1), &TabletHandler::timerCallback, this);
  node_->param(string(node_name)+"/dur_sentence_lock", dur_sentence_lock_, 60.0);

}

//########## CALLBACK: TIMER ###########################################################################################
void TabletHandler::timerCallback(const ros::TimerEvent &evt)
{
  if(hold_current_frag_.has_new_value()){
    cmr_msgs::ShowFragment new_frag;
    hold_current_frag_.get(new_frag);

    if(new_frag.name == "MenuHowto"){
        say("Hallo, ich bin Sobi. Ich kann dir Fragen zum Campusalltag beantworten. Wechsle dafür in das Robotermenü.");
    }
    else if(equalToStrAndNew(new_frag.name,"MenuRobot") || new_frag.name == "AfterStandby"){
        say("Drücke auf zuhören, um mit mir zu sprechen.");
    }
    /*
    else if(equalToStrAndNew(new_frag.name,"MenuMap")){
        say("Hier siehst du eine Standortkarte.");
    }*/
    else if(equalToStrAndNew(new_frag.name,"MenuTour")){
        say("Hier siehst du deinen aktuellen Standort auf dem Campus.");
    }
    else if(equalToStrAndNew(new_frag.name,"MenuTimetable")){
        say("Hier siehst du die Abfahrtszeiten der Busse vom Campus.");
    }
  /*  if(new_frag.name != "MenuHowto"){
        current_frag_ = new_frag;
    }*/
  }

}
void TabletHandler::say(const std::string &str){
    //If sentence not said, or time is passed store time and say sentence
    if (said_sentences_.find(str) == said_sentences_.end() ||
       (ros::Time::now() - said_sentences_[str]).toSec() > dur_sentence_lock_) {
      if(!speech_.getTTSClient()->isBusy(true)){
      		speech_.getTTSClient()->say(str,false);
      		said_sentences_[str] = ros::Time::now();
	}
    }

}
//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, node_name);

  ros::NodeHandle node_handle;
  TabletHandler tablet_handler(node_handle);

  ROS_INFO("Tablet Menu handler started...");
  ros::spin();

  return 0;
}
