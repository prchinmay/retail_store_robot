cmake_minimum_required(VERSION 3.0.2)
project(database)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  custom_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy custom_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
