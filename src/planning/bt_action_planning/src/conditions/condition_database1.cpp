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


#include <conditions/condition_database1.h>
#include <string>
#include "ros/ros.h"



// The condition database 1 will check if the created shoppinglist is of the same size as
// the shoppinglist_available. If this is true, the complete shopping list entered by the customer
// will be available in the store.
// Perfect! Now TIAGo can continue to create the order in which he will pick the products.



BT::ConditionDatabase1::ConditionDatabase1(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionDatabase1::~ConditionDatabase1() {}

BT::ReturnStatus BT::ConditionDatabase1::Tick()
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

        // Obtain the parameter /shopping_list_available, containing the avaiable products
        std::vector<std::string> list_available;
        if (ros::param::get("/shopping_list_available", list_available))
        {
          ROS_INFO("Obtained parameter /shopping_list_available");
        }

        // The condition which will be checked is if the avaible list of products is equal to
         // the request list of products
        // If true the status is set to SUCCESS
        // If false the status is set to FAILURE
        if (list.size() == list_available.size())
        {
            // If the list size of the shopping list is the same size as the available list
            // the condition will return SUCCESS
            set_status(BT::SUCCESS);
            std::cout << get_name() << " returning Success" << BT::SUCCESS << "!" << std::endl;
            ros::Duration(1).sleep();
            return BT::SUCCESS;
        }
        else
        {
            // Else the condition will return a failure
            set_status(BT::FAILURE);
            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
            ros::Duration(1).sleep();
            return BT::FAILURE;
        }
}


void BT::ConditionDatabase1::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
