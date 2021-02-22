#include "cmr_api/movement/moveable.h"
namespace cmr_api {
Moveable::Moveable()
{
    js_holder_.init("/joint_states");
}

double Moveable::getJointVal(int limbnum)
{
    string joint_name = getJointNameFromLimbNum(limbnum);
    sensor_msgs::JointStatePtr jsptr;
    js_holder_.get(jsptr);

    //Get iterator to element that contains joint name
    auto it = find(jsptr->name.begin(), jsptr->name.end(), joint_name);
    if(it != jsptr->name.end()){
        //Return joint value
        return jsptr->position[it - jsptr->name.begin()];
    }
    else{
        ROS_ERROR("Joint name not defined");
    }
}

bool Moveable::startMotionNeutral(int limbnum)
{
    auto cl_set = getNeutralMoversFromLimbNum(limbnum);
    bool ret = true;
    for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<NeutralMover> cl) mutable {ret = cl->start() && ret;});
    return ret;
}

void Moveable::stopMotionNeutral(int limbnum, double wait_time)
{
    auto cl_set = getNeutralMoversFromLimbNum(limbnum);
    for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<NeutralMover> cl){cl->stop();});
    abortMovements(limbnum);
    ros::Duration(wait_time).sleep();
}

bool Moveable::neutralActivated(int limbnum)
{
    auto cl_set = getNeutralMoversFromLimbNum(limbnum);
    bool ret = true;
    for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<NeutralMover> cl) mutable {ret = cl->isExecuting() && ret;});
    return ret;
}

void Moveable::abortMovements(int limbnum)
{
    auto tcl_set = getTrajClientsFromLimbNum(limbnum);
    for_each(tcl_set.begin(), tcl_set.end(),[=](shared_ptr<TrajectoryClient> cl)
    {
        cl->abortCurrentAction();
    });
}

set<shared_ptr<PositionClient> > Moveable::getPosClientsFromLimbNum(int limbnum)
{
    set<shared_ptr<cmr_api::PositionClient>> set;
    if(limbnum == cmr_msgs::LimbNum::EARS || limbnum == cmr_msgs::LimbNum::ARMS){
        set.insert(poscl_left_);
        set.insert(poscl_right_);

    }
    else if(limbnum == cmr_msgs::LimbNum::EAR_LEFT || limbnum == cmr_msgs::LimbNum::ARM_LEFT){
        set.insert(poscl_left_);

    }
    else if(limbnum == cmr_msgs::LimbNum::EAR_RIGHT|| limbnum == cmr_msgs::LimbNum::ARM_RIGHT){
        set.insert(poscl_right_);
    }
    else{
        ROS_ERROR_STREAM("Limb num "<<limbnum<<" not found");
    }
    return set;
}

set<shared_ptr<TrajectoryClient> > Moveable::getTrajClientsFromLimbNum(int limbnum)
{
    set<shared_ptr<cmr_api::TrajectoryClient>> set;
    if(limbnum == cmr_msgs::LimbNum::EARS || limbnum == cmr_msgs::LimbNum::ARMS){
        set.insert(trajcl_left_);
        set.insert(trajcl_right_);

    }
    else if(limbnum == cmr_msgs::LimbNum::EAR_LEFT || limbnum == cmr_msgs::LimbNum::ARM_LEFT){
        set.insert(trajcl_left_);

    }
    else if(limbnum == cmr_msgs::LimbNum::EAR_RIGHT|| limbnum == cmr_msgs::LimbNum::ARM_RIGHT){
        set.insert(trajcl_right_);
    }
    else{
        ROS_ERROR_STREAM("Limb num "<<limbnum<<" not found");
    }
    return set;
}

set<shared_ptr<NeutralMover> > Moveable::getNeutralMoversFromLimbNum(int limbnum)
{
    set<shared_ptr<cmr_api::NeutralMover>> set;
    if(limbnum == cmr_msgs::LimbNum::EARS || limbnum == cmr_msgs::LimbNum::ARMS){
        set.insert(nmov_left_);
        set.insert(nmov_right_);

    }
    else if(limbnum == cmr_msgs::LimbNum::EAR_LEFT || limbnum == cmr_msgs::LimbNum::ARM_LEFT){
        set.insert(nmov_left_);

    }
    else if(limbnum == cmr_msgs::LimbNum::EAR_RIGHT|| limbnum == cmr_msgs::LimbNum::ARM_RIGHT){
        set.insert(nmov_right_);
    }
    else{
        ROS_ERROR_STREAM("Limb num "<<limbnum<<" not found");
    }
    return set;
}

string Moveable::getJointNameFromLimbNum(int limbnum)
{
    switch(limbnum){
    case cmr_msgs::LimbNum::EAR_LEFT:{
        return "ear_left_joint";
        break;
    }
    case cmr_msgs::LimbNum::EAR_RIGHT:{
        return "ear_right_joint";
        break;
    }
    case cmr_msgs::LimbNum::ARM_LEFT:{
        return "arm_left_joint";
        break;
    }
    case cmr_msgs::LimbNum::ARM_RIGHT:{
        return "arm_right_joint";
        break;
    }
    default: return "";
    }
}
}
