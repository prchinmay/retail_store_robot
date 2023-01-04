#!/usr/bin/env python
import rospy
import actionlib
import numpy as np


from custom_msgs.srv import UpdateOrder, UpdateOrderResponse

# This function will sort the products so that TIAGO will travel the shortest route.

def sort_coordinates(req):

    res = UpdateOrderResponse()  # Create update order response message
    res.success = False          # Succes is set to false, until it is succesfully run


    # Retreiving the current shopping list
    shopping_list = req.input_list


    input_list = []
    output_list = []
    print("The current shopping list is")
    print(shopping_list)

    # Only if the shopping list is larger than 1 the items will be sorted
    if(len(shopping_list)>1):
        for item in shopping_list:
            print("the first item in the list is",item)
            # Here it will append the coordinates of the product
            input_list.append([item,rospy.get_param("/database/"+str(item)+"/pose/position/x"),rospy.get_param("/database/"+str(item)+"/pose/position/y")])

        # The function sorted will first sort in the x direction and then in the y direction.
        input_list = sorted(input_list, key=lambda k: [k[1],k[2]])


        print(" The list is now sorted")
        print(" the sorted list is now ", input_list)
        # Making the sorted output list
        output_list = [item[0] for item in input_list]
        print(" the output list is ", output_list)

        # Appending the output list to the response message
        res.output_list = output_list
        # Setting the response message succes to true
        res.success = True

    # If less than 2 products nothing happens
    else:
        res.output_list = req.input_list
        res.success = True

    return(res)
