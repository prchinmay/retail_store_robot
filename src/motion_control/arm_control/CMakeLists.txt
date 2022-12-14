cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    std_msgs
    message_generation
    geometry_msgs
    # custom_msgs
)

## Generate actions in the 'action' folder
# add_action_files(
# )

catkin_package(
)

include_directories(
)
