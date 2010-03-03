cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_gensrv()
rosbuild_genmsg()

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_library(door_functions src/door_functions.cpp)

rosbuild_add_gtest(test_door_functions  test/test_door_functions.cpp)
target_link_libraries(test_door_functions door_functions)

