cmake_minimum_required(VERSION 2.8.3)
project(cmr_description)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
    hector_gazebo_plugins
    tf
)
catkin_package(
   CATKIN_DEPENDS
   hector_gazebo_plugins
   hector_sensors_description
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(gtest)

  add_rostest_gtest(test_tf
    test/tf.test
    test/test_tf.cpp
  )

  target_link_libraries(
  test_tf
  ${catkin_LIBRARIES}
 )
 add_dependencies(
   test_tf
  ${catkin_EXPORTED_TARGETS}
)
endif()
