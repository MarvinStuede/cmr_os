#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
nav_msgs::Odometry lastOdom;

void odomCallback(const nav_msgs::Odometry &odom){
  lastOdom = odom;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber odom_sub = n.subscribe("/odom_ground_truth",10,&odomCallback);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();

  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    if(lastOdom.header.stamp.toSec() == 0) continue;

    //publish the transform over tf
    tf::Transform odom_trans;
    geometry_msgs::TransformStamped odom_trans_msg;


    odom_trans.setOrigin(tf::Vector3(lastOdom.pose.pose.position.x,lastOdom.pose.pose.position.y,lastOdom.pose.pose.position.z));
    odom_trans.setRotation(tf::Quaternion(lastOdom.pose.pose.orientation.x,lastOdom.pose.pose.orientation.y,lastOdom.pose.pose.orientation.z,lastOdom.pose.pose.orientation.w));

    tf::StampedTransform odom_trans_inv;
    odom_trans_inv.setData(odom_trans.inverse());
    odom_trans_inv.stamp_ = current_time;
    odom_trans_inv.frame_id_ = "base_link";
    odom_trans_inv.child_frame_id_ = "odom_ground_truth";

    tf::transformStampedTFToMsg(odom_trans_inv,odom_trans_msg);
    //send the transform
    odom_broadcaster.sendTransform(odom_trans_msg);

    r.sleep();
  }
}
