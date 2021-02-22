#pragma once
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
using namespace std;

class PoseRepublish
{
public:
    PoseRepublish(ros::NodeHandle &node_handle):nh_(&node_handle){
    sub_rob_pose_ = nh_->subscribe("/robot_pose",1,&PoseRepublish::subRobotPose, this);
    pub_rob_pose_ = nh_->advertise<geometry_msgs::PoseStamped>("/robot_pose_stamped",1);
    }

private:
    void subRobotPose(const geometry_msgs::Pose &pose){
        ps_.pose = pose;
        ps_.header.stamp = ros::Time::now();
        ps_.header.frame_id = "map";
        pub_rob_pose_.publish(ps_);
    }
    ros::NodeHandle* nh_;
    ros::Subscriber sub_rob_pose_;
    ros::Publisher pub_rob_pose_;
    geometry_msgs::PoseStamped ps_;

};
