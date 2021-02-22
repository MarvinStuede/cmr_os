/* *****************************************************************
 *
 * map_manager
 *
 * Copyright (c) 2019
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
 * @file   register_velodyne.cpp
 * @author Simon Ehlers (simon.ehlers@stud.uni-hannover.de)
 * @date   26.07.2019
 *
 * @brief  ROS-node for computing a depth image from the velodyne point cloud.
 * Therefore, the pointcloud is projected in the camera frame and then a dilation and vertical interpolation is executed for generating continous depth data.
 */

#include "cmr_driver/nodelets/register_velodyne.h"

namespace cmr_driver
{
//########## CONSTRUCTOR #####################################################################################
void RegisterVelodyne::onInit()
{
  // === PARAMETERS FROM LAUNCHFILE ===

  ros::NodeHandle & nh = getNodeHandle();
  ros::NodeHandle & pnh = getPrivateNodeHandle();
  std::string param;
  pnh.getParam("scan_topic", scan_topic_);
  pnh.getParam("cam_info_topic", cam_info_topic_);
  pnh.getParam("depth_image_topic", depth_image_topic_);
  pnh.getParam("camera_frame", camera_frame_);
  pnh.getParam("velodyne_frame", velo_frame_);
  pnh.getParam("horizontal_image", horizontal_image_);

  /*NODELET_INFO("Parameters:");
    std::cout << "scan_topic: " << scan_topic_ << "\n";
    std::cout << "cam_info_topic: " << cam_info_topic_ << "\n";
    std::cout << "depth_image_topic: " << depth_image_topic_ << "\n";
    std::cout << "camera_frame: " << camera_frame_ << "\n";
    std::cout << "horizontal_image: " << horizontal_image_ << "\n";*/

  // === PUBLISHERS ==

  image_transport::ImageTransport it(nh);
  depth_img16_pub_ = it.advertise(depth_image_topic_+"_16", 1); // 16 bits unsigned in mm
  depth_img32_pub_ = it.advertise(depth_image_topic_, 1);     // 32 bits float in meters

  bool got_transforms = false;
  do{
    NODELET_INFO("Waiting for static transforms");
    got_transforms = RegisterVelodyne::get_camera_transform();

  }
  while(!got_transforms && ros::ok());

  received_camera_info_ = false;

  // === SUBSCRIBERS ===
  pointcloud_subscriber_ = nh.subscribe(scan_topic_, 10, &RegisterVelodyne::subscriberCallback, this);
  camera_info_subscriber_ = nh.subscribe(cam_info_topic_, 10, &RegisterVelodyne::camInfoSubscriberCallback, this);

}

//########## CALLBACK: CAM INFO SUBSCRIBER CALLBACK ############################################################################
void RegisterVelodyne::camInfoSubscriberCallback(const sensor_msgs::CameraInfo &msg){
  //only fill in the information once at the beginning
  if(!received_camera_info_){
    f_x_ = msg.K.at(0);
    f_y_ = msg.K.at(4);
    c_x_ = msg.K.at(2);
    c_y_ = msg.K.at(5);

    imageSize_ = cv::Size(msg.width, msg.height);

    received_camera_info_ = true;
    NODELET_INFO("Received camera info!");
  }
}

//########## CALLBACK: SUBSCRIBER ############################################################################
void RegisterVelodyne::subscriberCallback(const sensor_msgs::PointCloud2Ptr &msg)
{

  //only compute depth image when camera info is available
  if(received_camera_info_){
    unsigned int i32_subs = depth_img32_pub_.getNumSubscribers();
    unsigned int i16_subs = depth_img16_pub_.getNumSubscribers();
    if(i32_subs || i16_subs)
    {

      //get pcl pointcloud from message
      pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg (*msg, *laserScan);

      // Executing the transformation
      //const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      //pcl::transformPointCloud (*laserScan, *laserScan_ptr, translation_, q_.cast<float>());

      cv::Mat  registered = cv::Mat::zeros(imageSize_, CV_32FC1);

      //############################# Inserted from rtabmap::util3d::projectCloudToCamera ########################
      rtabmap::Transform t = cameraTransform_.inverse();

      int count = 0;
      for(int i=0; i<(int)laserScan -> size(); ++i)
      {
        // Get 3D from laser scan
        pcl::PointXYZ ptScan = laserScan -> at(i);
        ptScan = rtabmap::util3d::transformPoint(ptScan, t);

        // re-project in camera frame
        float z = ptScan.z;
        float invZ = 1.0f/z;
        int dx = (f_x_*ptScan.x)*invZ + c_x_;
        int dy = (f_y_*ptScan.y)*invZ + c_y_;

        if(z > 0.0f && dx > 10 && dx <= registered.cols-10 && dy > 10 && dy <= registered.rows-10)
        {
          ++count;
          float &zReg = registered.at<float>(dy, dx);
          if(zReg == 0 || z < zReg)
          {
            zReg = z;
          }
        }
      }


      //############################### END INSERT ################################################

      //apply dilation
      registered = RegisterVelodyne::dilate_scanpoints(registered, 3);

      if(horizontal_image_){
        //vertical filling
        rtabmap::util3d::fillProjectedCloudHoles(registered, true, false);
      }
      else{
        //horizontal filling
        rtabmap::util3d::fillProjectedCloudHoles(registered, false, false);
      }

      std_msgs::Header scan_header = msg->header;
      std_msgs::Header depth_header;
      depth_header.seq = scan_header.seq;
      depth_header.stamp = scan_header.stamp;
      depth_header.frame_id = camera_frame_;

      cv_bridge::CvImage out_msg;
      out_msg.header   = depth_header; // Same timestamp and tf frame as input image
      out_msg.image    = registered; // Your cv::Mat

      sensor_msgs::ImagePtr img_to_pub = out_msg.toImageMsg();

      if(i32_subs)
      {
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depth_img32_pub_.publish(out_msg.toImageMsg());
      }
      if(i16_subs)
      {
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        out_msg.image = rtabmap::util2d::cvtDepthFromFloat(out_msg.image);
        depth_img16_pub_.publish(out_msg.toImageMsg());
      }
    }
  }
}

//########## DILATION SCANPOINTS ############################################################################
cv::Mat RegisterVelodyne::dilate_scanpoints(cv::Mat registered, int dilation_size){
  cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                               cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                               cv::Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  cv::dilate( registered, registered, element);
  return registered;
}

//########## GET CAMERA TRANSFORM ############################################################################
bool RegisterVelodyne::get_camera_transform()
{
  //transform listener
  tf::TransformListener listener;

  std::string targetFrameID = camera_frame_;
  std::string sourceFrameID = velo_frame_;
  tf::StampedTransform transformStamped;

  if(!cmr_os::util_tf::getTransformStamped(targetFrameID, sourceFrameID, transformStamped, listener,2.0)){
    return false;
  }
  tf::Transform tf;
  tf.setRotation(transformStamped.getRotation());
  tf.setOrigin(transformStamped.getOrigin());
  tf::Vector3 origin;
  tf::Quaternion rotation;
  origin = tf.getOrigin();
  rotation = tf.getRotation();

  const rtabmap::Transform cameraTransform_inverse(origin.getX(), origin.getY(), origin.getZ(),
                                                   rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW());
  cameraTransform_ = cameraTransform_inverse.inverse();

  return true;
}
}
