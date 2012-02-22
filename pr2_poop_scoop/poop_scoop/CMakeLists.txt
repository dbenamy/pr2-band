cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
rosbuild_gensrv()

#common commands for building c++ executables and libraries

rosbuild_add_library(poop_scooper src/arm.cpp src/head.cpp src/torso.cpp src/gripper.cpp src/visualize_arm.cpp)

rosbuild_add_executable(record_arm src/arm_motion_recorder.cpp)

rosbuild_add_executable(scooper src/scooper.cpp)
target_link_libraries(scooper poop_scooper)

#rosbuild_add_executable(scoop_placer src/scoop_placer.cpp)
#target_link_libraries(scoop_placer poop_scooper)

rosbuild_add_executable(fake_perception src/fake_perception.cpp)

#rosbuild_add_executable(ik_trajectory src/ik_trajectory.cpp)

#rosbuild_add_executable(test_arm src/arm.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#rosbuild_add_executable(send_song src/conductor.cpp)
#rosbuild_add_executable(drummer src/drummer.cpp)
#target_link_libraries(drummer conductor)
#rosbuild_add_executable(pianist src/pianist.cpp)
#target_link_libraries(pianist conductor)
