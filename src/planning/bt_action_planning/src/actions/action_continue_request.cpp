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


#include <actions/action_continue_request.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include "custom_msgs/ContinueRequest.h"
#include "custom_msgs/ContinueRequestResponse.h"



BT::ActionContinueRequest::ActionContinueRequest(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 1;
    thread_ = std::thread(&ActionContinueRequest::WaitForTick, this);
}

BT::ActionContinueRequest::~ActionContinueRequest() {}

void BT::ActionContinueRequest::WaitForTick()
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
        while (get_status() != BT::HALTED && i++ < time_)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id:" << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<custom_msgs::ContinueRequest>("continue_request/service");
            custom_msgs::ContinueRequest srv;
            srv.request.request =  true;

            // When not all of the items are available but one/some of them are a continue request message will be sent to the terminal
            // If the continue request is success TIAGo will continue his request otherwise an error message will be shown.

            bool success = false;
            while (success == false){

              if (client.call(srv))
              {
                success =  srv.response.success;  // Obtain whether the action
                                                  // performed by the service
                                                  // was succesfull.

                if (srv.response.continue_){
                  // If the client want to continue the task, whit the new list out
                  // of products, the /shopping_list is updated, only containing
                  // the available products.
                  std::vector<std::string> list_available;
                  if (ros::param::get("/shopping_list_available", list_available))
                  {
                    ROS_INFO("Obtained parameter /shopping_list");
                    ros::param::set("/shopping_list", list_available);
                    boolean_value_ = true;
                  }
                }
                else{
                  // If the customer does not want to continue, the shoppinglist
                  // is made empty.
                  // As the boolean_value_ is not set to true, this will mean
                  // that this action has failed. Returning back to the input_request
                  // action.
                  std::vector<std::string> list;
                  ros::param::set("/shopping_list", list);
                }
              }
              else
              {
                // Something went wront in the service call, so it will be tried
                // again.
                ROS_ERROR("Failed to call service input_request/service");
            }
          }
        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                // The action will be set to success if the customer decides
                // to continue with the new list of products.
                boolean_value_ = false; // This is set to false, as this is
                                        // necessary when the action is done
                                        // again in the futher.
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                // If the customer does not want to continue with the new list
                // of products, the action will be set to false.
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::ActionContinueRequest::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionContinueRequest::set_time(int time)
{
    time_ = time;
}



void BT::ActionContinueRequest::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
