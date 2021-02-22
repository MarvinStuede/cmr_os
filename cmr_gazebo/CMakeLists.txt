cmake_minimum_required(VERSION 2.8.3)
project(cmr_gazebo)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  tf
  roslib
  gazebo_ros
  nav_msgs
  octomap_server
  rgbd_launch
  controller_manager
  joint_state_controller
  joint_trajectory_controller
  position_controllers
  cmr_neo_ultrasonic
  gazebo_ros_control
  teleop_twist_keyboard
)
find_package(gazebo REQUIRED)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
add_executable(odom_gt_publisher
  src/odom_gt_publisher.cpp
)

add_dependencies(odom_gt_publisher ${PROJECT_NAME}_gencfg)

target_link_libraries(odom_gt_publisher
  ${catkin_LIBRARIES}
)
#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_gazebo
  test/gazebo.test
  test/test_gazebo.cpp
  )

  target_link_libraries(
  test_gazebo
  ${catkin_LIBRARIES}
 )
 add_dependencies(
   test_gazebo
  ${catkin_EXPORTED_TARGETS}
)

endif()
