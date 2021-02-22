#include <cmr_api/movement/neutral_mover.h>
namespace cmr_api{

bool NeutralMover::start()
{
    if(thread.joinable()) thread.join();
    thread = std::thread([=]{createMotionNeutral();});
    executing = true;
    return true;
}

void NeutralMover::createMotionNeutral()
{
    timer = nh.createTimer(ros::Duration(INTERVAL_NEUTRAL), &NeutralMover::motionNeutral, this);
    timer.start();
}

void NeutralMover::motionNeutral(const ros::TimerEvent&)
{
    if(!executing){
        timer.stop();
        return;
    }
    double time, angle;


    time = getRandInInterval(params_.min_time, params_.max_time);
    angle = getRandInInterval(params_.min_angle, params_.max_angle);
    if(traj_client){
        trajectory_msgs::JointTrajectory traj;
        traj_client->createTrajToAngle(traj,angle,time);
        traj_client->goTrajectory(traj);
    }
    else if(pos_client){
        pos_client->goToPos(angle);
    }
    else {
        ROS_ERROR("Neutral mover NULLPTR Error");
    }


}

double NeutralMover::getRandInInterval(double lower_bound, double upper_bound)
{
    random_device rd;
    default_random_engine re(rd());
    uniform_real_distribution<double> unif(lower_bound,upper_bound);
    return unif(re);
}



}
