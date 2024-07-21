cmake_minimum_required(VERSION 3.0.2)
project(lidar_opticalflow_fusion)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  "./devel/include"
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3/"
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(loff_node src/loff_node.cpp src/loff.cpp)
target_link_libraries(loff_node  ${catkin_LIBRARIES})


#  This is a debuging tool.
#  In order to debug LOFF non LiDAR degeneracy environments
# add_executable(fate_pose_correct_flag src/fate_pose_correct_flag.cpp)
# target_link_libraries(fate_pose_correct_flag  ${catkin_LIBRARIES})
