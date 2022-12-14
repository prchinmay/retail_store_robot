cmake_minimum_required(VERSION 3.0.2)
project(input_handling)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  custom_msgs
  actionlib
  move_base_msgs
  tf
  geometry_msgs
  
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PROJECT_DEPS}
)


include_directories(include ${catkin_INCLUDE_DIRS})



#add_executable(condition_move_base src/condition_move_base.cpp)
#target_link_libraries(condition_move_base  ${catkin_LIBRARIES})
#add_dependencies(condition_move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
