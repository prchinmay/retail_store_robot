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


#include <conditions/condition_handling_request.h>
#include <string>
#include "ros/ros.h"


// This condition is implemented so that TIAGo will know if an input request is being handled at the moment
// If this is the case, TIAGo can skip a large chunk of the behavior tree, containing the input request.


BT::ConditionHandlingRequest::ConditionHandlingRequest(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionHandlingRequest::~ConditionHandlingRequest() {}

BT::ReturnStatus BT::ConditionHandlingRequest::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        // Obtain the parameter if a request is being handled at the moment
        bool handling_request;
        if (ros::param::get("/handling_request", handling_request))
        {
          ROS_INFO("Obtained parameter /handling_request");
        }


        // The condition which will be checked if the robot currently is handling a request
        // If a robot is currently handling a request the boolean value handling_request will be true
        // The condition node will return SUCCESS
        // If no request is being handled at the moment the condition will return FAILURE
        if (handling_request)
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


void BT::ConditionHandlingRequest::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
