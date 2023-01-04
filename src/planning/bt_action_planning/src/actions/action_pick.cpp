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
#include <actions/action_pick.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <custom_msgs/PickAction.h>

BT::ActionPick::ActionPick(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 1;
    thread_ = std::thread(&ActionPick::WaitForTick, this);
}

BT::ActionPick::~ActionPick() {}

void BT::ActionPick::WaitForTick()
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
            // The parameter /hold/right_arm is set to false, this is default!
            ros::param::set("hold/right_arm", false);

            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));

            actionlib::SimpleActionClient<custom_msgs::PickAction>  ac("controls/pick_server", true);

            // Wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Connected with server");

            // Getting the shopping list
            std::vector<std::string> list_available;
            if (ros::param::get("/shopping_list_in_order", list_available))
            {
              ROS_INFO("Obtained parameter /shopping_list_in_order");
            }

            // Subtract the next item on the list, which has to be picked
            std::string item = list_available[0];

            // The code for the specific product,

            // The code for the specific product, the boolean value whether the
            // aruco will be used and the amount of products in stock are obtained.
            // In the case that use_aruco is true, this would mean a product will
            // be picked. And in the case it is false this would mean that a
            // basket will be picked or dropped. So in the case of this action,
            // use_aruco is ALWAYS true.
            int aruco_id; bool use_aruco; int amount;

            ros::param::get("/database/"+item+"/aruco_id", aruco_id);
            ros::param::get("/database/"+item+"/use_aruco", use_aruco);
            ros::param::get("/database/"+item+"/amount", amount);


            // We set the aruco id as follows, the first two digits will be the
            // digits of which product it will be (example milk = 33)
            // the last digit wil be the amount so if there are 4 packages of milk
            // first 334 will be picked than 333 etc.
            aruco_id = int(aruco_id*10+amount);


            // The goal containing the correct aruco marker is set
            custom_msgs::PickGoal goal;

            goal.use_aruco = use_aruco;
            goal.aruco_id = aruco_id;
            goal.gripper_side = "rightgrip";

            // Now the goal of picking the basket can be send.
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            // There will be waited, untill the action has either be executed
            // successfully, or the action was aborted. The state after the
            // action, will be stored in the paramter state.
            ac.waitForResult();

            actionlib::SimpleClientGoalState state = ac.getState();

            if (state != actionlib::SimpleClientGoalState::SUCCEEDED){
              // If the action was not succesfull, the boolean_value_ will not be
              // set to true, meaning the action was not succesfull.
              ROS_INFO("Action finished: %s",state.toString().c_str());
              finished = true;
            }
            else{
              // The action was succefull!
              // Now the boolean_value_ wil be set to true!
              boolean_value_ = false;

              // Finished is set to false, inorder to abort the while loop
              finished = true;

              // The parameter /hold/right_arm is set to true.
              // This means that a product has succefully be picked.
              // This product is necessary for the right arm state condition.
              ros::param::set("hold/right_arm", true);
            }

        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                // The action of picking and dropping the product has been done succefully
                boolean_value_ = false;   // boolean_value_ has been set to false
                                          // as this is necesarry whent the action
                                          // is executed again.

                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                // The action of picking and dropping the product was unsuccefull
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}


void BT::ActionPick::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionPick::set_time(int time)
{
    time_ = time;
}


void BT::ActionPick::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
