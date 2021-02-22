/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.

/**
 * @file   register_velodyne.h
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  ROS-nodelet for computing a depth image from the velodyne point cloud.
 * Therefore, the pointcloud is projected in the camera frame and then a dilation and vertical interpolation is executed for generating continous depth data.
 */

#ifndef REGISTER_VELODYNE_H
#define REGISTER_VELODYNE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/PointCloud2.h"
#include "fstream"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include "cmr_os/cmr_util_tf.h"
#include <pcl/common/transforms.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Geometry>
namespace cmr_driver
{
/**
 * @brief The RegisterVelodyne class:  ROS-node for computing a depth image from the velodyne point cloud.
 */
class RegisterVelodyne : public nodelet::Nodelet
{
public:

    /**
     * Constructor for initializing ROS-Node, subscriber and publisher.
     * Receive the static tranforms from tf tree necessary for creating depth image.
     * @brief RegisterVelodyne::RegisterVelodyne
     * @param node_handle (ros::NodeHandle)
     */
    RegisterVelodyne(){};

    virtual void onInit();
private:
    // node handle
    //ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber camera_info_subscriber_;

    image_transport::Publisher depth_img16_pub_;
    image_transport::Publisher depth_img32_pub_;


    // ############# CALLBACKS ####################
    /**
     * Callback to receive camera parameter fx, fy, cx, cy.
     * @brief RegisterVelodyne::camInfoSubscriberCallback
     * @param msg (sensor_msgs::CameraInfo)
     */
    void camInfoSubscriberCallback(const sensor_msgs::CameraInfo &msg);

    /**
     * Subscriber callback where PointCloud2 is received and the points are projected in the camera sensor frame.
     * Then a dilation and vertical interpolation is made for generating continous depth information.
     * The depth image is then publsihed.
     * @brief RegisterVelodyne::subscriberCallback
     * @param msg (const sensor_msgs::PointCloud2)
     */
    void subscriberCallback(const sensor_msgs::PointCloud2Ptr &msg);


    // ############### METHODS ###################
    /**
     * Dilation of scan points to fill horizontal holes between the scan points.
     * So, it is possible to conduct a vertical interpolation in the next step.
     * @brief RegisterVelodyne::dilate_scanpoints
     * @param registered (cv::Mat)
     * @param dilation_size (int)
     * @return registered (cv::Mat)
     */
    cv::Mat dilate_scanpoints(cv::Mat registered, int dilation_size);

    /**
     * Get transform from base_link to velodyne by using a transform listener.
     * @brief RegisterVelodyne::get_velodyne_transform
     */
    //bool get_velodyne_transform();

    /**
     * Get transform from velodyne to cam_front_color_optical_frame by using a transform listener.
     * @brief RegisterVelodyne::get_camera_transform
     */
    bool get_camera_transform();


    // parameters
    Eigen::Quaterniond q_;
    Eigen::Vector3f translation_;
    rtabmap::Transform cameraTransform_;
    std::string scan_topic_;
    std::string cam_info_topic_;
    std::string depth_image_topic_;
    std::string camera_frame_;
    std::string velo_frame_;
    bool horizontal_image_;

    //camera parameter from camera info
    bool received_camera_info_;
    cv::Size imageSize_;
    float f_x_;
    float f_y_;
    float c_x_;
    float c_y_;
};

PLUGINLIB_EXPORT_CLASS(cmr_driver::RegisterVelodyne, nodelet::Nodelet)
}
#endif // REGISTER_VELODYNE_H
