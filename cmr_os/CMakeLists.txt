cmake_minimum_required(VERSION 2.8.3)
project(cmr_os)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmr_description
  roscpp
  geometry_msgs
  nav_msgs
  rostest
  tf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
    src/cmr_holder.cpp
    src/cmr_util_tf.cpp
    src/cmr_util_filter.cpp
    src/cmr_util_prob.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(pose_republish
  src/pose_republish.cpp
)
target_link_libraries(pose_republish
  ${catkin_LIBRARIES}
)
add_executable(shutdown_node
  src/shutdown_node.cpp
)
target_link_libraries(shutdown_node
  ${catkin_LIBRARIES}
)
#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(
    ${PROJECT_NAME}-test
    test/cmr_util_test.cpp
  )

  target_link_libraries(
    ${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  add_dependencies(${PROJECT_NAME}-test
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )

endif()
