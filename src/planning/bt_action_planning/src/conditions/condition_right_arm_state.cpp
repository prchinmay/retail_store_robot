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


#include <conditions/condition_right_arm_state.h>
#include <string>
#include "ros/ros.h"


// The condition will check if TIAGo is currently holding a product or not
// If TIAGo is holding a product it will return SUCCESS
// If not then it will return FAILURE


BT::ConditionRightArmState::ConditionRightArmState(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionRightArmState::~ConditionRightArmState() {}

BT::ReturnStatus BT::ConditionRightArmState::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        // Obtain the parameter /hold containing the requested products
        // This parameter is set to true if the action pick (product) was
        // executed succefully. And was set to failure, if it had aborted
        // This condition node is merly meant to control the bahavior of the
        // tree, meaning if the picking action fails, the second level fallback
        // will try to drive back to the home location and drop the basket.
        // Howevever, as this condition will fail, this is not possible. So
        // instead the picking action will be executed again.
        bool hold;
        if (ros::param::get("/hold/right_arm", hold))
        {
          ROS_INFO("Obtained parameter /hold/right_arm");
        }

        // If the obtaine paramter is true, the conditions will return succes.
        // And if the parameter is false, the condition will return failure.
        if (hold)
        {
            set_status(BT::SUCCESS);
            std::cout << get_name() << " returning Success" << BT::SUCCESS << "!" << std::endl;
            ros::Duration(1).sleep();
            return BT::SUCCESS;
        }
        else
        {
            set_status(BT::FAILURE);
            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
            ros::Duration(1).sleep();
            return BT::FAILURE;
        }
}


void BT::ConditionRightArmState::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
