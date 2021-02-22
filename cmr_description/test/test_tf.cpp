#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <thread>
using namespace std;
using namespace std::this_thread;

//Test if tf transform between base_link and several other link exists
TEST(TestSuite, StaticTFTest){
  double wait_dur = 3.0;
  tf::TransformListener listener;
  std::string frame_base = "base_link";
  std::vector<std::string> frames;
  frames.push_back("cam_back_link");
  frames.push_back("cam_front_link");
  frames.push_back("velodyne");
  frames.push_back("imu_sensor_link");
  frames.push_back("ear_left_link");
  frames.push_back("ear_right_link");
  frames.push_back("arm_left_link");
  frames.push_back("arm_right_link");
  frames.push_back("wheel_left_link");
  frames.push_back("wheel_right_link");
  frames.push_back("scanner_front_link");

  bool tf_available = true;
  for(const auto & frame : frames){
    if(!tf_available) break;
    tf_available = tf_available && listener.waitForTransform(frame_base,frame,ros::Time(0),ros::Duration(wait_dur));
  }

  EXPECT_TRUE(tf_available);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "TfTestNode");
  testing::InitGoogleTest(&argc, argv);

  thread t([]{while(ros::ok()) ros::spin();});

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
