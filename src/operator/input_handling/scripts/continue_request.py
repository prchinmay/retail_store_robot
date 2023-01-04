#!/usr/bin/env python
import rospy
import actionlib

from custom_msgs.srv import ContinueRequest, ContinueRequestResponse

# The continue request function will only come into play if one or multiple products
# are not in stock but at least one is
# It will ask the client to continue y or n


def continue_request(req):
    res = ContinueRequestResponse() # Create ContinueRequestResponse message
    res.success = False             # Succes is set to false, until it is succesfully run

    # Check wether the input request is true
    if req.request:

        # Read the parameters about the requested list and the available list
        shopping_list_available = rospy.get_param("/shopping_list_available")
        shopping_list = rospy.get_param("/shopping_list")

        # Print the obtained parameters and state the choice menu
        for i in range(2):  print('')
        print('Your request products are: ')
        print(shopping_list)
        print ("Not all products are in stock, do you want to continue with the following list:")
        print (shopping_list_available)
        print('If you want to continue press [y] if you want to abort press [n]')

        # While the input is not succesfull, ask for an input of the customer
        while not res.success:
            input_ = raw_input().strip().lower()

            if input_ == 'y':
                res.continue_ = True
                res.success = True
                print('YES')
            elif input_ == 'n':
                res.continue_ = False
                res.success = True
                print('NO')
            else:
                print('')
                print('The input is not one of the following options: Yes [y] or No [n]')
                print('Please try again!')

        rospy.loginfo('%s: Succeeded' % rospy.get_name())

    # If input request is false give an error message
    else:
        rospy.logerr('%s: Failed' % rospy.get_name())

    return res
