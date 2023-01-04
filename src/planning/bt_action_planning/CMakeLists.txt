cmake_minimum_required(VERSION 3.0.2)
project(bt_action_planning)

find_package(catkin REQUIRED COMPONENTS
  behavior_tree_core
  behavior_tree_leaves
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  roslaunch
  genmsg
  custom_msgs
  move_base_msgs
  tf
  geometry_msgs
)



add_definitions(-Wall -lglut -lGL -std=c++0x)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs behavior_tree_core behavior_tree_leaves
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# FIND GTest
#########################################################
find_package(GTest)
include_directories(${GTEST_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} include)

file(GLOB_RECURSE BTHeadLibrary include/*.h)

set(BTSrcLibrary
src/actions/action_input_request.cpp
src/actions/action_move_base.cpp
src/actions/action_continue_request.cpp
src/actions/action_check_database.cpp
src/actions/action_update_next_product.cpp
src/actions/action_pick.cpp
src/actions/action_return_to_home.cpp
src/actions/action_update_order.cpp
src/actions/action_drop_basket.cpp
src/actions/action_pick_basket.cpp
src/conditions/condition_input_request.cpp
src/conditions/condition_database1.cpp
src/conditions/condition_database2.cpp
src/conditions/condition_move_base.cpp
src/conditions/condition_return_to_home.cpp
src/conditions/condition_handling_request.cpp
src/conditions/condition_shoppinglist.cpp
src/conditions/condition_right_arm_state.cpp
)

add_library(${PROJECT_NAME} ${BTSrcLibrary} ${BTHeadLibrary})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
add_dependencies(${PROJECT_NAME} behavior_tree_core_generate_messages_cpp)

add_executable(control_tree src/control_tree.cpp)
target_link_libraries(control_tree
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${GTEST_LIBRARIES})
