#include "cmr_os/cmr_util_tf.h"
// Bring in gtest
#include <gtest/gtest.h>
#define D_ERR 0.000001
// Declare a test

struct poses{
  geometry_msgs::Pose p1, p2, p3, p4, p5, p6, p7;
  poses(){
    p1.position.x = 1.0;
    p1.position.y = 2.0;
    p1.position.z = 3.0;

    p2.position.x = 2.0;
    p2.position.y = 4.0;
    p2.position.z = 6.0;

    p3.position.x = 1.0;
    p3.position.y = 2.0;

    p4.position.x = 0.5;
    p4.position.y = 3.0;

    p5.position.x = 3.0;
    p5.position.y = 2.0;

    p6.position.x = -p3.position.x;
    p6.position.y = -p3.position.y;

    p7.position.x = -p4.position.x;
    p7.position.y = -p4.position.y;

  }
}poses;
TEST(TestSuite, tfEuclidianDist)
{

  EXPECT_DOUBLE_EQ(3.741657386773941,cmr_os::util_tf::euclidianDistBetweenPoses(poses.p1,poses.p2));
  EXPECT_EQ(cmr_os::util_tf::euclidianDistBetweenPoses(poses.p1,poses.p2),
            cmr_os::util_tf::euclidianDistBetweenPoses(poses.p2,poses.p1));
  EXPECT_EQ(0.0,cmr_os::util_tf::euclidianDistBetweenPoses(poses.p1,poses.p1));


}
TEST(TestSuite, inCircle)
{
  double r = 1.5;
  EXPECT_TRUE(cmr_os::util_tf::pointInCircle(poses.p3,poses.p4,r));
  EXPECT_FALSE(cmr_os::util_tf::pointInCircle(poses.p5,poses.p4,r));
  EXPECT_TRUE(cmr_os::util_tf::pointInCircle(poses.p5,poses.p5,r));

  EXPECT_TRUE(cmr_os::util_tf::pointInCircle(poses.p6,poses.p7,r));
}

TEST(TestSuite, pathLength)
{
  nav_msgs::Path path;
  geometry_msgs::PoseStamped ps1,ps2,ps3,ps4,ps5,ps6,ps7;
  ps1.pose = poses.p1;
  ps2.pose = poses.p2;
  ps3.pose = poses.p3;
  ps4.pose = poses.p4;
  ps5.pose = poses.p5;
  ps6.pose = poses.p6;
  ps7.pose = poses.p7;
  path.poses.push_back(ps1);
  path.poses.push_back(ps2);
  path.poses.push_back(ps3);
  path.poses.push_back(ps4);
  path.poses.push_back(ps5);
  path.poses.push_back(ps6);
  path.poses.push_back(ps7);
  double l = 20.730286254766213;
  EXPECT_DOUBLE_EQ(l,cmr_os::util_tf::getPathLength(path));
  path.poses.clear();
  path.poses.push_back(ps7);
  path.poses.push_back(ps6);
  path.poses.push_back(ps5);
  path.poses.push_back(ps4);
  path.poses.push_back(ps3);
  path.poses.push_back(ps2);
  path.poses.push_back(ps1);
  EXPECT_DOUBLE_EQ(l,cmr_os::util_tf::getPathLength(path));

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
