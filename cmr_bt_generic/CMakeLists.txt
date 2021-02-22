cmake_minimum_required(VERSION 2.8.3)
project(cmr_bt_generic)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  cmr_api
  neo_msgs
  behaviortree_cpp_v3
)

# find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cmr_bt_general_nodes cmr_bt_social_nodes cmr_bt_system_nodes
    CATKIN_DEPENDS cmr_api behaviortree_cpp_v3
    )

include_directories(
  include
  ${cmr_api_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

function(set_plugin_properties plugin)
    target_link_libraries(${plugin} PRIVATE
      ${BEHAVIOR_TREE_LIBRARY}
      ${catkin_LIBRARIES}
    )
    target_include_directories(${plugin} PUBLIC ${catkin_INCLUDE_DIRS})
    target_compile_definitions(${plugin} PRIVATE  BT_PLUGIN_EXPORT )
    set_target_properties(${plugin} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
endfunction()

add_library(cmr_bt_general_nodes SHARED
    include/cmr_bt_generic/general.h
    src/general.cpp
)

set_plugin_properties(cmr_bt_general_nodes)

add_library(cmr_bt_system_nodes SHARED
    include/cmr_bt_generic/system_nodes.h
    src/system_nodes.cpp
)

set_plugin_properties(cmr_bt_system_nodes)

add_library(cmr_bt_social_nodes SHARED
    include/cmr_bt_generic/social_nodes.h
    src/social_nodes.cpp
)

set_plugin_properties(cmr_bt_social_nodes)

