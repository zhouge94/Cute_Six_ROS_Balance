cmake_minimum_required(VERSION 2.8.3)
project(cutesix_2dnav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

