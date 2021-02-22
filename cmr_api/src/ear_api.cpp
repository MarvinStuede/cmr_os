#include <cmr_api/ear_api.h>

using namespace std;
cmr_api::EarAPI::~EarAPI()
{

}

void cmr_api::EarAPI::init(ros::NodeHandle &nh)
{
  nh_ = &nh;
  params_left_.nh = &nh;
  params_right_.nh = &nh;

  //Set names of controllers
  //TODO: Read names from config file
  params_left_.pos_ctrl_name = "ear_left_position_controller";
  params_left_.traj_ctrl_name = "ear_left_trajectory_controller";
  params_right_.pos_ctrl_name = "ear_right_position_controller";
  params_right_.traj_ctrl_name = "ear_right_trajectory_controller";
  params_left_.joint_name = "ear_left_joint";
  params_right_.joint_name = "ear_right_joint";

  //Define Service Client to switch between the different controllers
  //TODO: Read topic from config
  serv_switch_controller_ = nh_->serviceClient<controller_manager_msgs::SwitchController>("/sobi/controller_manager/switch_controller");

  params_left_.serv_switch_controller_ = &serv_switch_controller_;
  params_right_.serv_switch_controller_ = &serv_switch_controller_;

  trajcl_left_ = std::make_shared<TrajectoryClient>(params_left_);
  poscl_left_ = std::make_shared<PositionClient>(params_left_);
  trajcl_right_ = std::make_shared<TrajectoryClient>(params_right_);
  poscl_right_ = std::make_shared<PositionClient>(params_right_);

  nmov_left_ = std::make_shared<NeutralMover>(trajcl_left_);
  nmov_right_ = std::make_shared<NeutralMover>(trajcl_right_);

  //Set Trajectory controllers as standard controllers
  trajcl_left_->activate();
  trajcl_right_->activate();

}


bool cmr_api::EarAPI::motionWiggle(double duration, int limbnum, double max_angle_wiggle, double time_per_repeat, double time_to_stay)
{
  auto cl_set = getTrajClientsFromLimbNum(limbnum);
  int sgn = 1;
  u_int repeats = int(duration/(3*time_per_repeat + 2*time_to_stay));
  if(repeats == 0) repeats = 1;
  bool ret = true;
  for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<TrajectoryClient> cl) mutable {
      ret = cl->motionWiggle(repeats,sgn*max_angle_wiggle,time_per_repeat,time_to_stay) && ret;
      sgn*=-1;
  });
  return ret;
}

bool cmr_api::EarAPI::motionHang(double dur_total, double dur_movement, int limbnum, double max_angle_hang)
{
  auto cl_set = getTrajClientsFromLimbNum(limbnum);
  bool ret = true;
  for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<TrajectoryClient> cl) mutable {
      ret = cl->moveToPosAndStay(max_angle_hang,dur_total,dur_movement) && ret;
  });
  return ret;
}

bool cmr_api::EarAPI::motionGoTo(double dur_movement, int limbnum, double max_angle_hang)
{
  auto cl_set = getTrajClientsFromLimbNum(limbnum);
  bool ret = true;
  for_each(cl_set.begin(), cl_set.end(),[=](shared_ptr<TrajectoryClient> cl) mutable {
      ret = cl->moveToPos(max_angle_hang, dur_movement) && ret;
  });
  return ret;
}
