cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_85_gripper_visualization)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  legacy
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
