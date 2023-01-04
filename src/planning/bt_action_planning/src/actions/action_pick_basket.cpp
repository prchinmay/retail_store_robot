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

#include <actions/action_pick_basket.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <custom_msgs/PickAction.h>



BT::ActionPickBasket::ActionPickBasket(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 1;
    thread_ = std::thread(&ActionPickBasket::WaitForTick, this);
}

BT::ActionPickBasket::~ActionPickBasket() {}

void BT::ActionPickBasket::WaitForTick()
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
        while (get_status() != BT::HALTED && finished == false)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Define the picking action server, which controls the movment of the arms.
            actionlib::SimpleActionClient<custom_msgs::PickAction>  ac("controls/pick_server", true);

            // Wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for the controls/pick_server action server to come up");
            }
            ROS_INFO("Connected with server");

            // The aruco id and the boolean value whether the aruco will be used
            // are obtained. In the case that use_aruco is true, this would mean
            // a product will be picked. And in the case it is false this would
            // mean that a basket will be picked or dropped. So in the case of
            // this action, use_aruco is ALWAYS false.
            int aruco_id; bool use_aruco;

            ros::param::get("/basket/aruco_id", aruco_id);
            ros::param::get("basket/use_aruco", use_aruco);

            // Obtain the hard-coded position where the the arm hould move to
            // in order to pick the basket.
            float pose_x; float pose_y; float pose_z; float orient_x; float orient_y; float orient_z; float orient_w;

            ros::param::get("/basket/pose/position/x", pose_x);
            ros::param::get("/basket/pose/position/y", pose_y);
            ros::param::get("/basket/pose/position/z", pose_z);
            ros::param::get("/basket/pose/orientation/x", orient_x);
            ros::param::get("/basket/pose/orientation/y", orient_y);
            ros::param::get("/basket/pose/orientation/z", orient_z);
            ros::param::get("/basket/pose/orientation/w", orient_w);

            // The goal for the picking of the basket is here set. This is
            // based on the obtained information about the basket.
            custom_msgs::PickGoal goal;

            goal.use_aruco = use_aruco;
            goal.aruco_id = aruco_id;
            goal.gripper_side = "leftgrip";  // For the handling of the basket
                                             // The left arm will ALWAYS be used.

            goal.pose.position.x = pose_x;
            goal.pose.position.y = pose_y;
            goal.pose.position.z = pose_z;

            goal.pose.orientation.x = orient_x;
            goal.pose.orientation.y = orient_y;
            goal.pose.orientation.z = orient_z;
            goal.pose.orientation.w = orient_w;

            // Now the goal of picking the basket can be send.
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            // There will be waited, untill the action has either be executed
            // successfully, or the action was aborted. 
            ac.waitForResult();

            actionlib::SimpleClientGoalState state = ac.getState();

            if (state != actionlib::SimpleClientGoalState::SUCCEEDED){\
              // If the action was not succesfull, the boolean_value_ will not be
              // set to true, meaning the action was not succesfull.
              ROS_INFO("Action finished: %s",state.toString().c_str());
              finished = true;
            }
            else{
              // The action was succefull!
              // Now the boolean_value_ wil be set to true!
              boolean_value_ = true;

              // Finished is set to false, inorder to abort the while loop
              finished = true;

              // Setting the request handling condition to true so that the input request subtree will be skipped in the future.
              ros::param::set("/handling_request", true);

              // Setting the paramter /hold/left_arm to true, as TIAGo is now holding the basket.
              ros::param::set("/hold/left_arm", true);
            }

        }


        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                // The action of picking the basket has been done succefully
                boolean_value_ = false;   // boolean_value_ has been set to false
                                          // as this is necesarry whent the action
                                          // is executed again.
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                // The action of dropping the basket was not successfull!
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::ActionPickBasket::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionPickBasket::set_time(int time)
{
    time_ = time;
}



void BT::ActionPickBasket::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
