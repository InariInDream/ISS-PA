cmake_minimum_required(VERSION 3.0.2)
project(wheel_imu_deadreckoning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/calc_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
