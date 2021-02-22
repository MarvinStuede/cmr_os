/* *****************************************************************
 *
 * rotate_image_nodelet
 *
 * Copyright (c) 2020
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
 * @file   rotate_image_nodelet.cpp
 * @author Marvin Stuede (Marvin.Stuede@imes.uni-hannover.de)
 * @date   05/2020
 *
 * @brief  Nodelet to rotate an image for 90 deg
 */


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <memory>

using namespace sensor_msgs;
using namespace message_filters;

namespace cmr_driver{


typedef sync_policies::ApproximateTime<Image, CameraInfo> ImgInfoPolicy;

class RotateImage : public nodelet::Nodelet
{
public:


     
    RotateImage():
	sync_info_(0)
{};
    virtual ~RotateImage(){
	delete sync_info_;

    };
    virtual void onInit(){
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();

    	image_transport::ImageTransport it(nh);

	//Image transport subscribers
	sub_img_.subscribe(it, "image_in",1);
	sub_info_.subscribe(nh, "info_in", 1);

	//Publishers for rotated image and info
	pub_img_ = it.advertise("image_out",10);
	pub_info_ = nh.advertise<CameraInfo>("info_out",10);

        pnh.param<std::string>("encoding", encoding_, "rgb8");
        pnh.param<std::string>("rot_frame_id", rot_frame_, "cam_back_rotated_color_optical_frame");

	//Synchronize images with infos
	sync_info_ =   new Synchronizer<ImgInfoPolicy>(ImgInfoPolicy(10), sub_img_, sub_info_);
	sync_info_->registerCallback(boost::bind(&RotateImage::cbImgCamInfo, this, _1, _2));

     }
private:

    ros::Publisher pub_info_;

    Synchronizer<ImgInfoPolicy> *sync_info_;

    image_transport::Publisher pub_img_;

    message_filters::Subscriber<CameraInfo> sub_info_;
    image_transport::SubscriberFilter sub_img_;


    double angle_ = 90.;
    std::string rot_frame_;
    std::string encoding_;



 void cbImgCamInfo(const ImageConstPtr& img, const CameraInfoConstPtr& cam_info){

      ImagePtr img_rot;
      if(!rotateImage(img,img_rot,encoding_)){
	return;
      }
      CameraInfo cam_info_rot = convertCameraInfo(*cam_info,rot_frame_);
      pub_img_.publish(*img_rot);
      pub_info_.publish(cam_info_rot);

  }


  CameraInfo convertCameraInfo(const CameraInfo &orig, const std::string new_frame_id){
     CameraInfo cam_info_rot = orig;
      cam_info_rot.header.frame_id = new_frame_id;
      cam_info_rot.K.at(0) = orig.K.at(4);
      cam_info_rot.K.at(2) = orig.K.at(5);
      cam_info_rot.K.at(5) = orig.K.at(2);
      cam_info_rot.K.at(4) = orig.K.at(0);
      cam_info_rot.height = orig.width;
      cam_info_rot.width = orig.height;
      return cam_info_rot;
  }


  bool rotateImage(const ImageConstPtr &img_in, ImagePtr &img_out, const std::string &encoding){
        cv_bridge::CvImageConstPtr cv_ptr;
	cv_bridge::CvImage cv_out;
	
        try{
           cv_ptr = cv_bridge::toCvShare(img_in, encoding);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
	}


     // get rotation matrix for rotating the image around its center in pixel coordinates
      cv::Point2f center((cv_ptr->image.cols-1)/2.0, (cv_ptr->image.rows-1)/2.0);
      cv::Mat rot = cv::getRotationMatrix2D(center, angle_, 1.0);
      // determine bounding rectangle, center not relevant
      cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), cv_ptr->image.size(), angle_).boundingRect2f();
      // adjust transformation matrix
      rot.at<double>(0,2) += bbox.width/2.0 - cv_ptr->image.cols/2.0;
      rot.at<double>(1,2) += bbox.height/2.0 - cv_ptr->image.rows/2.0;

      cv::warpAffine(cv_ptr->image, cv_out.image, rot, bbox.size());

      cv_out.header   = img_in->header; // Same timestamp and tf frame as input image
      cv_out.header.frame_id = rot_frame_;
      cv_out.encoding = img_in->encoding;
 
      img_out = cv_out.toImageMsg();
      return true;
  }
 
};

PLUGINLIB_EXPORT_CLASS(cmr_driver::RotateImage, nodelet::Nodelet)
}
