#include "cmr_driver/pcl_repub.h"

void PCLRepub::callback(const sensor_msgs::PointCloud2ConstPtr &pc_in)
{
    sensor_msgs::PointCloud2 pc_out;
    pcl_ros::transformPointCloud("velodyne",*pc_in,pc_out,listener_);
    pub_pc_.publish(pc_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_repub_node");

    ros::NodeHandle nh("~");

    PCLRepub pcl_repub(nh);
    std::cout<<"Velodyne Republisher is Spinning"<<std::endl;
    ros::spin();

    return 0;
}

