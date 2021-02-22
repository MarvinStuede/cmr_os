/* *****************************************************************
Copyright (c) 2020, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   pcl_repub.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  ROS node to republish a pointcloud in different frame. Workaround for RTAB-Map, which expects pointcloud in base_link frame
 */

#pragma once
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"

class PCLRepub{
public:
    PCLRepub(ros::NodeHandle &node_handle):  node_(&node_handle)
    {
        sub_pc_ = node_->subscribe("/velodyne_points",1, &PCLRepub::callback,this);
        pub_pc_ = node_->advertise<sensor_msgs::PointCloud2>("/velodyne_points_vel", 10);

    }

    private:
        void callback(const sensor_msgs::PointCloud2ConstPtr &pc_in);
        tf::TransformListener listener_;
        ros::Publisher pub_pc_;
        ros::Subscriber sub_pc_;
        ros::NodeHandle* node_;

};
