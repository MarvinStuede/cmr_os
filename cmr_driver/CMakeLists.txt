cmake_minimum_required(VERSION 2.8.3)
project(cmr_driver)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
find_package(PCL REQUIRED)
FIND_PACKAGE(RTABMap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  message_filters
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  rtabmap_ros
  pointcloud_to_laserscan
  nodelet
  cmr_os
)

catkin_package(
    LIBRARIES cmr_driver_nodelets
    CATKIN_DEPENDS nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(pcl_repub
  src/pcl_repub.cpp
  include/cmr_driver/pcl_repub.h
)

add_library(cmr_driver_nodelets
  src/nodelets/register_velodyne.cpp
  include/cmr_driver/nodelets/register_velodyne.h
  src/nodelets/rotate_image_nodelet.cpp
 )

target_link_libraries(pcl_repub
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
        add_dependencies(cmr_driver_nodelets ${catkin_EXPORTED_LIBRARIES})
endif()

target_link_libraries(cmr_driver_nodelets
  ${catkin_LIBRARIES}
  ${RTABMap_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS cmr_driver_nodelets
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES nodelet_plugins.xml
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

option(DOWNLOAD_BAG_DATA "Download bag files with data for tests"  OFF)
if(CATKIN_ENABLE_TESTING)

  if(DOWNLOAD_BAG_DATA)
     execute_process(COMMAND rosrun cmr_driver get_bag.sh)
  endif()

  find_package(rostest REQUIRED)

  add_rostest(test/sensor_nodelets.test)

endif()
