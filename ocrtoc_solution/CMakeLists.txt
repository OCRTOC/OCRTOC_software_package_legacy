cmake_minimum_required(VERSION 3.0.2)
project(ocrtoc_solution)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  ocrtoc_task
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocrtoc_solution
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  controller
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/commit_solution.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
