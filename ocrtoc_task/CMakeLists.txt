cmake_minimum_required(VERSION 2.8.3)
project(ocrtoc_task)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)
add_action_files(FILES Clean.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_e_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

install(DIRECTORY
  action
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  scripts/task_evaluation.pyc
  scripts/trigger.py
  scripts/trigger_and_evaluation.py
  launch/timed_roslaunch.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
