cmake_minimum_required(VERSION 2.8.3)
project(cmr_api)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  cmr_os
  cmr_msgs
  control_msgs
  controller_manager_msgs
  actionlib
  actionlib_msgs
  roslib
)

find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS cmr_os control_msgs cmr_msgs roslib
    )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${cmr_os_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/ear_api.cpp
    src/arm_api.cpp
    src/speech_api.cpp
    src/led_api.cpp
    src/action_wrapper.cpp
    src/movement/moveable.cpp
    src/movement/neutral_mover.cpp
    include/cmr_api/action_wrapper.h
    include/cmr_api/movement/position_client.h
    include/cmr_api/movement/trajectory_client.h
    include/cmr_api/move_base_client.h
    include/cmr_api/call_supervisor_client.h
    ${Boost_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(sentiment_executor
  src/nodes/sentiment_executor.cpp
  include/cmr_api/sentiment_executor.h
)
add_executable(tablet_menu_handler
  src/nodes/tablet_menu_handler.cpp
  include/cmr_api/tablet_menu_handler.h
)
target_link_libraries(tablet_menu_handler
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
target_link_libraries(sentiment_executor
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
