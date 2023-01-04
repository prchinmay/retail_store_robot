/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <actions/action_move_base.h>
#include <string>
#include "ros/ros.h"
#include <iostream>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


BT::ActionMoveBase::ActionMoveBase(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 3;
    thread_ = std::thread(&ActionMoveBase::WaitForTick, this);
}

BT::ActionMoveBase::~ActionMoveBase() {}

void BT::ActionMoveBase::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        bool finished = false; // Value to control the running of the while loop

        // Perform action...
        while (get_status() != BT::HALTED && finished == false) //&& i++ < time_)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));

            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  ac("move_base", true);


            // Wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Connected with server");

            // Obtain the list of products in order

            std::vector<std::string> list_available;
            if (ros::param::get("/shopping_list_in_order", list_available))
            {
              ROS_INFO("Obtained parameter /shopping_list_in_order");
            }

            // Subtract the first product on the list
            std::string item = list_available[0];


            // Here the pose and the orientation in quaternion will be retrieved from the database
            float pose_x; float pose_y; float pose_z; float orient_x; float orient_y; float orient_z; float orient_w;

            ros::param::get("/database/"+item+"/pose/position/x", pose_x);
            ros::param::get("/database/"+item+"/pose/position/y", pose_y);
            ros::param::get("/database/"+item+"/pose/position/z", pose_z);
            ros::param::get("/database/"+item+"/pose/orientation/x", orient_x);
            ros::param::get("/database/"+item+"/pose/orientation/y", orient_y);
            ros::param::get("/database/"+item+"/pose/orientation/z", orient_z);
            ros::param::get("/database/"+item+"/pose/orientation/w", orient_w);

            move_base_msgs::MoveBaseGoal goal;

            //Here the goal will be set to the location of the product from the shopping list
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = pose_x;
            goal.target_pose.pose.position.y = pose_y;
            goal.target_pose.pose.position.z = pose_z;

            goal.target_pose.pose.orientation.x = orient_x;
            goal.target_pose.pose.orientation.y = orient_y;
            goal.target_pose.pose.orientation.z = orient_z;
            goal.target_pose.pose.orientation.w = orient_w;


            // Now the goal will be send to the server
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            // The next step is to wait for the result of the action service
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
              // If the action return succefully, the while loop will be closed
              // And the boolean_value_ will be set to true
              ROS_INFO("Hooray, the base moved to the correct location");
              finished = true;
              boolean_value_ = true;
            }
            else // if the action is unsuccefull, it will be tried again!
              ROS_INFO("The base failed to move to the correct location");
        }
        if (get_status() != BT::HALTED)
        {
            finished = false;
            if (boolean_value_)
            {
                // If the action has been execute succefully, and the base has return home
                // this action will be succefull!
                boolean_value_ = false;   // boolean_value_ has been set to false
                                          // as this is necesarry whent the action
                                          //  is executed again.
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                // In theory the action will never be set to failuer, unless the while loop
                // if being stoped, while not finished. In this case, this node is set to failure.
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::ActionMoveBase::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionMoveBase::set_time(int time)
{
    time_ = time;
}



void BT::ActionMoveBase::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
