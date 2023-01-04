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


#include <conditions/condition_shoppinglist.h>
#include <string>
#include "ros/ros.h"

// The condition which will be checked is if there is a next product to be picked by Tiago. The if statement
// checks if the list is bigger than 0, meaning if there are some more products to be picked.
// If this is true the condition will set its status to FAILURE, this is done such that the sequence loop
// will be run again; Tiago will drive to a product and pick it.
// If the statement is false, the condition will set its status to SUCCESS, meaning there are no more
// products to be picked.



BT::ConditionShoppinglist::ConditionShoppinglist(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionShoppinglist::~ConditionShoppinglist() {}

BT::ReturnStatus BT::ConditionShoppinglist::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        // Obtain the parameter /shopping_list containing the requested products
        std::vector<std::string> list;
        if (ros::param::get("/shopping_list_in_order", list))
        {
          ROS_INFO("Obtained parameter /shopping_list_in_order");
        }

        // If the list is greater than 0, there are still some product which have to be picked.
        // if the list is smaller than 0 TIAGO will return home and drop of the basket
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


void BT::ConditionShoppinglist::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
