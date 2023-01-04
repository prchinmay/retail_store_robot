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


#include <conditions/condition_return_to_home.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

// This condition will check if TIAGo is at its home position or not.
// The home position is stored in parameters so it can be easily compared to the current location


BT::ConditionReturnToHome::ConditionReturnToHome(std::string name) : ConditionNode::ConditionNode(name)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = false;
}

BT::ConditionReturnToHome::~ConditionReturnToHome() {}

BT::ReturnStatus BT::ConditionReturnToHome::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        ros::NodeHandle node;             // Create NodeHandle object
        tf::TransformListener listener;   // Creaet Transformlistenere object

        // The condition which will be check in this condition is whether Tiago is at the base position.
        // The following while loop will be run untill the position of Tiago is found.

        bool found_pose = false;          // The position is initally not found, so this is set to false
                                          // As soon as the position is found, this is set to true
                                          // This will have as effect that the while loop will not be executed anymore
        while (found_pose == false){
          tf::StampedTransform transform;   // Create StampedTransform object, this will contain the position of Tiago

          // Try to obtain the transfrom between the map (world) and base_footprint (Tiago), if this is done succesfully
          // found_pose is set to true.
          // If it is not possible to obtain, an error message will pop up and the while loop will continue from the beginning
          try{
            listener.lookupTransform("/map", "/base_footprint",
                                     ros::Time(0), transform);
            found_pose = true;
          }
          catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
          }
          // Based on the found position of tiago in the world frame, some float parameter are given an value
          float pos_x =  transform.getOrigin().x();
          float pos_y =  transform.getOrigin().y();
          float pos_z =  transform.getOrigin().z();

          float orient_x  = transform.getRotation().x();
          float orient_y  = transform.getRotation().y();
          float orient_z  = transform.getRotation().z();
          float orient_w  = transform.getRotation().w();

          printf("point of Tiag0 in frame of map  Position(x:%f y:%f z:%f) Orientation(Position(x:%f y:%f z:%f w:%f))\n",
                 pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w);

          // Define some floats for the goal position of tiago
          float base_pos_x; float base_pos_y; float base_pos_z; float base_orient_x; float base_orient_y; float base_orient_z; float base_orient_w;

          // Obtain the specified base position of tiago, this will be the desired location of tiago at this moment.
          ros::param::get("/base/pose/position/x", base_pos_x);
          ros::param::get("/base/pose/position/y", base_pos_y);
          ros::param::get("/base/pose/position/z", base_pos_z);
          ros::param::get("/base/pose/orientation/x",base_orient_x);
          ros::param::get("/base/pose/orientation/y",base_orient_y);
          ros::param::get("/base/pose/orientation/z",base_orient_z);
          ros::param::get("/base/pose/orientation/w",base_orient_w);

          // Define the maximum error for the position and orientations
          float error_pos = 0.02;
          float error_or = 0.01;

          // Check if the position of the tiago is the same position (with a small deflection possible), as the base position.
          // If this is the case, the boolean_value is set to true
          if (abs(base_pos_x - pos_x) < error_pos && abs(base_pos_y - pos_y) < error_pos && abs(base_pos_z - pos_z) < error_pos &&
            abs(orient_x - base_orient_x)< error_or && abs(orient_y - base_orient_y)< error_or && abs(orient_w - base_orient_w)< error_or){
              boolean_value_ = true;
              printf("point of Tiag0 in frame of map  Position(x:%f y:%f z:%f) Orientation(Position(x:%f y:%f z:%f w:%f))\n",
                     pos_x-base_pos_x, pos_y-base_pos_y, pos_z-base_pos_z, orient_x-base_orient_x, orient_y-base_orient_y, orient_z-base_orient_z, orient_w-base_orient_w);

          }
          }

        // Here it will be checked whether the boolean_value is set to true. Initally this was set to false (in line 23)
        // If the boolean_value_ is true the return status is SUCCESS
        // If the boolean_value_ is false the return status is FALSE
        if (boolean_value_)
        {
          boolean_value_ = false; // Boolean_value_ is set to false again, this is done because if this condition is tested again
                                  // it would not automatically be set to false, as was done when initalizing an object of this class
                                  // in the behavior tree file. If it would not be false, than this node would in the future always
                                  // return SUCCESS, even if Tiago is not at the correct position.
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


void BT::ConditionReturnToHome::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}
