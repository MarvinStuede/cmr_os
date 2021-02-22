#pragma once
#include "ros/ros.h"
#include <controller_manager_msgs/SwitchController.h>

/**
 * @brief The CtrlManagement struct
 * Stores joint controller names and further information
 */
struct CtrlManagement{
  ros::NodeHandle* nh;
  ros::ServiceClient* serv_switch_controller_;
  std::string traj_ctrl_name;
  std::string pos_ctrl_name;
  std::string joint_name;

  /**
   * @brief switchController
   * Switches a controller. Eg. from position to joint trajectory controller
   * @param ctrl_disable Controller name to disable
   * @param ctrl_enable Controller name to enable
   * @return true if controller switched successfully
   */
  bool switchController(const std::string &ctrl_disable, const std::string &ctrl_enable)
  {

    ROS_INFO_STREAM("Activating "<<ctrl_enable);
    controller_manager_msgs::SwitchController serv_switch;

    serv_switch.request.stop_controllers.push_back(ctrl_disable);
    serv_switch.request.start_controllers.push_back(ctrl_enable);
    serv_switch.request.strictness = 1;

    if(!serv_switch_controller_->exists()){
      ROS_ERROR("Switch controller service not available");
      return false;
    }

    serv_switch_controller_->call(serv_switch);
    if(serv_switch.response.ok){
      ROS_INFO_STREAM(ctrl_enable << " activated.");
      return true;
    }
    else return false;

  }
};
inline double deg2rad(double dval){return dval/180. * M_PI;}
inline double rad2deg(double rval){return rval/M_PI * 180.;}

/**
 * @brief Check if position is reached
 * @param current_pos Current position
 * @param goal_pos Goal position
 * @param eps threshold to check for
 * @return true if difference below threshold
 */
inline bool posReached(double current_pos, double goal_pos, double eps = 0.03){
  return fabs(current_pos - goal_pos) < eps;
}

