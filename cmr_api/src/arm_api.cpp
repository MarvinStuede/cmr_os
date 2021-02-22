#include <cmr_api/arm_api.h>

cmr_api::ArmAPI::ArmAPI()
{

}

cmr_api::ArmAPI::~ArmAPI()
{

}

void cmr_api::ArmAPI::init(ros::NodeHandle &nh)
{

    nh_ = &nh;
    params_left_.nh = &nh;
    params_right_.nh = &nh;

    //Set names of controllers
    //TODO: Read names from config file
    params_left_.pos_ctrl_name = "arm_left_position_controller";
    params_left_.traj_ctrl_name = "";
    params_right_.pos_ctrl_name = "arm_right_position_controller";
    params_right_.traj_ctrl_name = "";
    params_left_.joint_name = "arm_left_joint";
    params_right_.joint_name = "arm_right_joint";

    poscl_left_ = std::make_shared<PositionClient>(params_left_);
    poscl_right_ = std::make_shared<PositionClient>(params_right_);
    cmr_api::NMovParams nmovparams;
    nmovparams.min_angle = 0.;
    nmovparams.max_angle = 45.;

    nmov_left_ = std::make_shared<NeutralMover>(poscl_left_,nmovparams);
    nmov_right_ = std::make_shared<NeutralMover>(poscl_right_,nmovparams);

    poscl_left_->activate();
    poscl_right_->activate();

}

bool cmr_api::ArmAPI::moveToPosAndStay(double angle, double dur_stay, int limbnum)
{
    auto cl_set = getPosClientsFromLimbNum(limbnum);
    if(cl_set.empty()) return false;
    bool ret = true;
    for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<PositionClient> cl) mutable {
        ret = cl->moveToPosAndStay(angle, dur_stay) && ret;
    });
    return ret;
}

bool cmr_api::ArmAPI::moveToPos(double angle, int limbnum)
{
    auto cl_set = getPosClientsFromLimbNum(limbnum);
    if(cl_set.empty()) return false;
    bool ret = true;
    for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<PositionClient> cl) mutable {
        ret = cl->goToPos(angle) && ret;
    });
    return ret;
}

