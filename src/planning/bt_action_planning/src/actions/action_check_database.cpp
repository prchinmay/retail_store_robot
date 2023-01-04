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


#include <actions/action_check_database.h>
#include <string>
#include "ros/ros.h"
#include <iostream>
#include "custom_msgs/CheckDatabase.h"
#include "custom_msgs/CheckDatabaseResponse.h"



BT::ActionCheckDatabase::ActionCheckDatabase(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = true;
    time_ = 1;
    thread_ = std::thread(&ActionCheckDatabase::WaitForTick, this);
}

BT::ActionCheckDatabase::~ActionCheckDatabase() {}

void BT::ActionCheckDatabase::WaitForTick()
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

            // Creating the vector shopping list and loading it with the parameters
            std::vector<std::string> list;
            if (ros::param::get("/shopping_list", list))
            {
              ROS_INFO("Obtained parameter /shopping_list");
            }
            // Initializing the node and service client check database
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<custom_msgs::CheckDatabase>("/database/check_service");
            custom_msgs::CheckDatabase srv;

            // checking if the shopping list is available
            if (ros::param::get("/shopping_list", srv.request.products ))
            {
              ROS_INFO("Obtained parameter /shopping_list");
            }

            // When a request is received there will check if every item from the shopping list exists in the database
            if (client.call(srv))
            {
              bool success =  srv.response.success;
              ROS_INFO_STREAM(success);
              if (success == false){
                boolean_value_ = false;
                break;
              }

              // Print the existing products
              int idx=1;
              for (bool item : srv.response.exist){
                if (idx == int( srv.response.exist.size())){
                  std::cout << item << std::endl;
                }
                else{
                  std::cout << item << " ";
                }
                ++idx;
              }

              // Print the products which are in stock
              idx=1;
              for (bool item : srv.response.in_stock){
                  if (idx == int(srv.response.in_stock.size())){
                    std::cout << item << std::endl;
                  }
                  else{
                    std::cout << item << " ";
                  }
                  ++idx;
                }

                // Add the products in stock to a vector of strings.
                std::vector<std::string> list_available;
                idx = 0;
                for (bool item : srv.response.in_stock){
                  if (item ==1 ){
                    list_available.push_back(list[idx]);
                  }
                  ++idx;
                }
                // The parameters shopping list available will be set.
                // This paramter will be used during the check if the product
                // request correspond to the available products.
                n.setParam("/shopping_list_available", list_available);
            }
            else
            {
              // If request is unsuccefull, the boolean_value_ will be set to False.
              // This results in a failure of the action
              boolean_value_ = false;
              ROS_ERROR("Failed to call service database/check_service");
            }

        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                // If the databse was checked for the request products the
                // action will return to be succefull.
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                // If it was not possible to send a request to the
                // /database/check_service or the request send a failure back
                // then this action has failed.
                boolean_value_ = true;  // Set back to true for the next execution
                                        // of this node
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::ActionCheckDatabase::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::ActionCheckDatabase::set_time(int time)
{
    time_ = time;
}



void BT::ActionCheckDatabase::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
