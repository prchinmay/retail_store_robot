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


#include <actions/action_update_next_product.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include "custom_msgs/ContinueRequest.h"
#include "custom_msgs/ContinueRequestResponse.h"
#include "custom_msgs/UpdateDatabase.h"
#include "custom_msgs/UpdateDatabaseResponse.h"



BT::ActionUpdateNextProduct::ActionUpdateNextProduct(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 1;
    thread_ = std::thread(&ActionUpdateNextProduct::WaitForTick, this);
}

BT::ActionUpdateNextProduct::~ActionUpdateNextProduct() {}

void BT::ActionUpdateNextProduct::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);
        // Perform action...
        int i = 0;
        while (get_status() != BT::HALTED && boolean_value_ == false && i++ < time_)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));

            ros::NodeHandle n;

            // Create client to update the database
            ros::ServiceClient client = n.serviceClient<custom_msgs::UpdateDatabase>("/database/update_service");
            custom_msgs::UpdateDatabase srv;

            // Obtain the paramter /shopping_list_in_order
            std::vector<std::string> list_available;
            if (ros::param::get("/shopping_list_in_order", list_available))
            {
              ROS_INFO("Obtained parameter /shopping_list_in_order");
            }

            // Take the first item from the list, its database has to be updated
            std::string item = list_available[0];
            //printf("Next product to find %s" , item);
            std::cout << "Next product to find" << item;

            // Update the database
            srv.request.product = item;
            bool success;
            if (client.call(srv))
            {
              int amount = srv.response.amount;
              success =  srv.response.success;
              //ROS_INFO("Amount of stock of %s is %i", item, amount);
              std::cout << "amounf of stock of" << item<< "is"<< amount;
            }

            // A new list is created, here the first item of the old list is not added
            std::vector<std::string> new_list;
            for (unsigned idx = 1; idx < list_available.size(); ++idx){
              new_list.push_back(list_available[idx]);
            }

            printf("list size: %zu", sizeof(new_list));

            // The new list is set to the parameter /shopping_list_in_order
            ros::param::set("/shopping_list_in_order", new_list);

            // Making sure the list size is smaller than the old list and that the updating of the database
            // was done succesfully
            if (new_list.size() != list_available.size() && success == true){
              boolean_value_ = true;
            }

        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                boolean_value_ = false;
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::ActionUpdateNextProduct::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionUpdateNextProduct::set_time(int time)
{
    time_ = time;
}



void BT::ActionUpdateNextProduct::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
