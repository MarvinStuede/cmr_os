/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   cmr_util_tf.h
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   2019
*
* @brief  Utility functions for Transforms
*/
#pragma once
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include <random>

namespace cmr_os{
namespace util_tf{
using namespace std;

/**
* @brief Transform pose from source to target frame
* @param pose
* @param sourceFrame
* @param targetFrame
* @return true if successful
*/
bool transformPose(geometry_msgs::Pose &pose, std::string sourceFrame, std::string targetFrame, const tf::TransformListener &transformer);


/**
 * @brief Transform stamped pose
 * @param pose
 * @param targetFrame
 * @return true if successful
 */
bool transformPoseStamped(geometry_msgs::PoseStamped &pose, std::string targetFrame, const tf::TransformListener &transformer);

/**
 * @brief Get a transform
 * @param target_frame
 * @param source_frame
 * @param tf to write to
 * @param listener
 * @param wait_time 0 as default
 * @return true if transform received
 */
bool getTransform(string target_frame, string source_frame, tf::Transform &tf, tf::TransformListener &listener, double wait_time = 0);

/**
 * @brief Get a transform
 * @param target_frame
 * @param source_frame
 * @param tf
 * @param listener
 * @param wait_time
 * @return true if transform received
 */
bool getTransformStamped(string target_frame, string source_frame, tf::StampedTransform &tf, tf::TransformListener &listener, double wait_time = 0);


/**
 * @brief Convert psoe to transform
 * @param pose
 * @return  transform
 */
tf::Transform getTransformFromPose(const geometry_msgs::Pose &pose);

/**
 * @brief Convert transform to pose
 * @param pose
 * @return  transform
 */
geometry_msgs::Pose getPoseFromTransform(const tf::Transform &tf);

/**
 * @brief Calculate a path with move_base "GetPlan" Service
 * @param path object to write path to
 * @param start Start of path
 * @param goal Goal of path
 * @param sclient
 * @return
 */
bool calculateGeometricPath(nav_msgs::Path &path, const tf::Vector3 &start, const tf::Vector3 &goal, ros::ServiceClient &sclient, std::string frame_id = "map");

/**
 * @brief Get the length of a path
 * Calculates the euclidian distance between consecutive path positions
 * @param path
 * @return length
 */
double getPathLength(const nav_msgs::Path &path);

/**
 * @brief Print a pose to terminal
 * @param pose
 * @param name
 */
void printPose(const geometry_msgs::Pose &pose,string name);

/**
 * @brief Calculate the euclidian distance between poses
 * @param pose1
 * @param pose2
 * @return distance
 */
double euclidianDistBetweenPoses(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

/**
 * @brief Check if a point lies within a circle. Only uses x and y component
 * @param circle_center
 * @param point
 * @param radius
 * @return True if point lies in circle
 */
bool pointInCircle(const tf::Vector3 &circle_center, const tf::Vector3 &point, double radius);
/**
 * @brief Check if a point lies within a circle. Only uses x and y component
 * @param circle_center
 * @param point
 * @param radius
 * @return True if point lies in circle
 */
bool pointInCircle(const geometry_msgs::Pose &circle_center, const geometry_msgs::Pose &point, double radius);

template<typename T> T getRandInInterval(T lower_bound, T upper_bound)
{
    std::random_device rd;
    std::default_random_engine re(rd());
    std::uniform_real_distribution<T> unif(lower_bound,upper_bound);
    return unif(re);
}
};
};
