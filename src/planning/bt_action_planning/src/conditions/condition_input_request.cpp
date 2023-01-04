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


#include <conditions/condition_input_request.h>
#include <string>
#include "ros/ros.h"


// This condition will check if the shopping list is filled with at least one product.
// If this list is <= 0, a new list has to be created so the action input_request will start
// Now you will be asked to make a new shopping list inside the terminal




BT::ConditionInputRequest::ConditionInputRequest(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionInputRequest::~ConditionInputRequest() {}

BT::ReturnStatus BT::ConditionInputRequest::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        // Obtain the parameter /shopping_list containing the requested products
        std::vector<std::string> list;
        if (ros::param::get("/shopping_list", list))
        {
          ROS_INFO("Obtained parameter /shopping_list");
        }

        // The condition which will be checked is if the list of request products is bigger than 0, in other words if it is not empty.
        // If true the status is set to SUCCESS
        // If false the status is set to FAILURE
        if (list.size() > 0)
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




void BT::ConditionInputRequest::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
