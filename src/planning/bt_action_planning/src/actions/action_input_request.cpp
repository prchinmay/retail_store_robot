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


#include <actions/action_input_request.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include "custom_msgs/InputRequest.h"
#include "custom_msgs/InputRequestResponse.h"



BT::ActionInputRequest::ActionInputRequest(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 1;
    thread_ = std::thread(&ActionInputRequest::WaitForTick, this);
}

BT::ActionInputRequest::~ActionInputRequest() {}

void BT::ActionInputRequest::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        int i = 0;
        // Perform action...
        while (get_status() != BT::HALTED && i++ < time_)
        {
            bool success = false; // Boolean is set to false on default
            
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Node and service client input request will be initialized
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<custom_msgs::InputRequest>("input_request/service");

            // The type of the srv will be Input Request from custom messages
            custom_msgs::InputRequest srv;
            srv.request.request =  true;

            while (success == false){

                // Here the shopping list will be created from the terminal.
                // After every product you will have to press "enter"
                // At the end of the list you will have to press "q"
                // This request is handeld by the input_request/service

              if (client.call(srv))
              {
                // The boolean succes will be update based on the response of the service
                success =  srv.response.success;
                ROS_INFO_STREAM(success);

                // If the service call was succesfull, the shopping list will be updated:
                if (success){
                    std::vector<std::string> list = srv.response.list;
                    ROS_INFO("Number of models: %zd", list.size());

                    // Print the requested products
                    int idx=1;
                    for (std::string item : list){
                      if (idx == int(list.size())){
                        std::cout << item << std::endl;
                      }
                      else{
                        std::cout << item << " ";
                      }
                      ++idx;
                    }

                    // The shopping list parameter will be set after the list is completed
                    n.setParam("/shopping_list", list);

                    // Set boolean_value_ to true, as the aciton was succesfull
                    boolean_value_ = true;
                }
              }
              else
              {
                ROS_ERROR("Failed to call service input_request/service");
            }

          }


        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                // If the service call was succefull, and a list of products was
                // retrieved, this action will be succefull!
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

void BT::ActionInputRequest::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionInputRequest::set_time(int time)
{
    time_ = time;
}



void BT::ActionInputRequest::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
