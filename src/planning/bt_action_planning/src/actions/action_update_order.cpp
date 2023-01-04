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


#include <actions/action_update_order.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include "custom_msgs/UpdateOrder.h"
#include "custom_msgs/UpdateOrderResponse.h"



BT::ActionUpdateOrder::ActionUpdateOrder(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 1;
    thread_ = std::thread(&ActionUpdateOrder::WaitForTick, this);
}

BT::ActionUpdateOrder::~ActionUpdateOrder() {}

void BT::ActionUpdateOrder::WaitForTick()
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
        while (get_status() != BT::HALTED &&  finished == false)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Initializing the node and service client check database
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<custom_msgs::UpdateOrder>("/update_order/service");

            // Create an object which will be send trough the service call
            custom_msgs::UpdateOrder srv;

            // checking if the shopping list is available and store it in the the service request
            if (ros::param::get("/shopping_list_available", srv.request.input_list ))
            {
              ROS_INFO("Obtained parameter /shopping_list_available");
            }

            // Sending the object to the service call
            if (client.call(srv))
            {
              // If succesfull the service call will update the shopping list, such that the shortest
              // path will be responed.
              bool success =  srv.response.success;
              ROS_INFO_STREAM(success);

              finished = true;

              // if the service call response with unsuccessfull, then the action will return as a failer
              if (success != true){
                break;
              }

              // if the service call response is succesfull, the action will return as succefull.
              boolean_value_ = true;

              // Update the shopping_list_in_order with the retrieved shopping list
              // This order will be used to pick the different products
              n.setParam("/shopping_list_in_order", srv.response.output_list);
            }
            else
            {
              // If the service call is unsuccefull try again!
              ROS_ERROR("Failed to call service ");
            }

        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                // The updating of the order has been done succefully!
                boolean_value_ = false; // This is set to false, as this is
                                        // necessary when the action is done
                                        // again in the future.
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                // If the updating of the order is unsuccefull, set the status to failure.
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::ActionUpdateOrder::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionUpdateOrder::set_time(int time)
{
    time_ = time;
}



void BT::ActionUpdateOrder::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
