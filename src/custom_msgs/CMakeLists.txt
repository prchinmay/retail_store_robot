cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  message_generation
  geometry_msgs
)

add_action_files(
  FILES
  Pick.action
)

add_service_files(
  FILES
  CheckDatabase.srv
  UpdateDatabase.srv
  InputRequest.srv
  ContinueRequest.srv
  UpdateOrder.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
