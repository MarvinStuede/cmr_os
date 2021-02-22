#include <cmr_os/pose_republish.h>
//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_pose_republish");

    ros::NodeHandle node_handle;
    PoseRepublish pose_repub(node_handle);
    ros::spin();

    return 0;
}
