#include "cmr_os/cmr_util_tf.h"

namespace cmr_os{
namespace util_tf{
using namespace std;

bool transformPoseStamped(geometry_msgs::PoseStamped &pose, std::string targetFrame, const tf::TransformListener &transformer){
    if(pose.header.frame_id == targetFrame)
        return true;

    bool transformAvailable = transformer.waitForTransform(targetFrame,pose.header.frame_id,ros::Time(0),ros::Duration(2.0));

    if(transformAvailable){
        try{
            transformer.transformPose(targetFrame,ros::Time(0),pose,pose.header.frame_id,pose);
            return true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }
    }
    else{
        ROS_WARN("Transform from '%s' to '%s' not available!",pose.header.frame_id.c_str(),targetFrame.c_str());
        return false;
    }
}
bool getTransform(string target_frame, string source_frame, tf::Transform &tf, tf::TransformListener &listener, double wait_time)
{
    tf::StampedTransform tf_stamped;
    bool received = getTransformStamped(target_frame,source_frame,tf_stamped,listener,wait_time);
    if(!received) return false;
    tf.setRotation(tf_stamped.getRotation());
    tf.setOrigin(tf_stamped.getOrigin());
}
bool getTransformStamped(string target_frame, string source_frame, tf::StampedTransform &tf, tf::TransformListener &listener, double wait_time){
    bool transform_available = true;

    if(wait_time > 0)
        transform_available = listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(wait_time));


    if(transform_available){
        try{
            listener.lookupTransform(target_frame,source_frame,ros::Time(0),tf);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }
    }
    else{
        ROS_WARN("Transform from '%s' to '%s' not available!",source_frame.c_str(),target_frame.c_str());
        return false;
    }
    return true;
}

tf::Transform getTransformFromPose(const geometry_msgs::Pose &pose)
{
    tf::Transform transform;
    tf::Vector3 origin;
    tf::Quaternion rotation;

    origin[0] = pose.position.x;
    origin[1] = pose.position.y;
    origin[2] = pose.position.z;
    rotation.setX(pose.orientation.x);
    rotation.setY(pose.orientation.y);
    rotation.setZ(pose.orientation.z);
    rotation.setW(pose.orientation.w);

    transform.setOrigin(origin);
    transform.setRotation(rotation);

    return transform;
}
geometry_msgs::Pose getPoseFromTransform(const tf::Transform &tf){
    geometry_msgs::Pose pose;
    tf::Vector3 origin = tf.getOrigin();
    tf::Quaternion rotation = tf.getRotation();

    pose.position.x = origin[0];
    pose.position.y = origin[1];
    pose.position.z = origin[2];

    pose.orientation.w = rotation.getW();
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();

    return pose;
}

void printPose(const geometry_msgs::Pose &pose,string name)
{
    ROS_INFO("------------- %s ---------------",name.c_str());
    ROS_INFO("p_x: %f, p_y: %f, p_z: %f",pose.position.x,pose.position.y,pose.position.z);
    ROS_INFO("r_x: %f, r_y: %f, r_z: %f, r_w: %f",pose.orientation.x,pose.orientation.y,pose.orientation.z, pose.orientation.w);
    ROS_INFO("------------- %s ---------------",name.c_str());

}

bool transformPose(geometry_msgs::Pose &pose, std::string sourceFrame, std::string targetFrame, const tf::TransformListener &transformer){
    geometry_msgs::PoseStamped poseS;
    poseS.pose = pose;
    poseS.header.frame_id = sourceFrame;
    poseS.header.stamp = ros::Time::now();
    bool success = transformPoseStamped(poseS,targetFrame, transformer);
    if(!success) return false;
    pose = poseS.pose;
    return true;
}
double euclidianDistBetweenPoses(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2)
{
    double x = pose2.position.x - pose1.position.x;
    double y = pose2.position.y - pose1.position.y;
    double z = pose2.position.z - pose1.position.z;
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

bool pointInCircle(const tf::Vector3 &circle_center, const tf::Vector3 &point, double radius){
    bool in_circle = pow(circle_center.getX() - point.getX(),2) + pow(circle_center.getY() - point.getY(),2) < (radius * radius);
    return in_circle;
}

bool pointInCircle(const geometry_msgs::Pose &circle_center, const geometry_msgs::Pose &point, double radius){
    tf::Vector3 tcircle_c, t_point;
    tcircle_c.setX(circle_center.position.x);
    tcircle_c.setY(circle_center.position.y);
    tcircle_c.setZ(circle_center.position.z);
    t_point.setX(point.position.x);
    t_point.setY(point.position.y);
    t_point.setZ(point.position.z);

    return pointInCircle(tcircle_c, t_point, radius);
}


double getPathLength(const nav_msgs::Path &path){

    double l = 0;
    for(auto it =std::next(path.poses.begin(),1) ; it!= path.poses.end(); ++it){
        l += euclidianDistBetweenPoses(std::prev(it)->pose,it->pose);
    }
    return l;

}

bool calculateGeometricPath(nav_msgs::Path &path, const tf::Vector3 &start, const tf::Vector3 &goal, ros::ServiceClient &sclient, std::string frame_id){
    geometry_msgs::PoseStamped pose_s, pose_g;
    pose_s.header.frame_id = frame_id;
    pose_s.header.stamp = ros::Time::now();
    pose_s.pose.orientation.w = 1;

    pose_g = pose_s;

    pose_s.pose.position.x = start.getX();
    pose_s.pose.position.y = start.getY();
    pose_g.pose.position.x = goal.getX();
    pose_g.pose.position.y = goal.getY();


    nav_msgs::GetPlan temp_plan;
    temp_plan.request.tolerance = 0.3;
    temp_plan.request.start  = pose_s;
    temp_plan.request.goal  = pose_g;

    if(!sclient) return false;
    if(sclient.call(temp_plan.request,temp_plan.response))
    {
        if(temp_plan.response.plan.poses.empty()) return false;
        path = temp_plan.response.plan;
        return true;

    }
    else{
        return false;
    }

}

}

}
