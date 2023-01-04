#!/usr/bin/env python
import rospy
import actionlib

from custom_msgs.srv import InputRequest, InputRequestResponse

# Here the input request of the client is created
# All the print functions will be shown in the terminal so the client will know exactly what to do



def input_request(req):
    res = InputRequestResponse()    # Create InputRequestResponse message
    res.success = False             # Succes is set to false, until it is succesfully run

    database = rospy.get_param('/database') # Retrieve the database
    # Check wether the input request is true
    if req.request:

        # Ask customer for list of products
        for i in range(100):  print('')
        print ("Type products which you want to get, followed by [ENTER].")
        print ("If you entered all product press [q], followed by [ENTER].")
        print ("If you wish to abort press [a], followed by [ENTER].")
        print ("If you want to know which products our supermarket has, type [h] followed by [ENTER].")
        for i in range(5):  print('')
        input_ = raw_input().strip().lower()

        while input_ != 'q' and input_!='a':
            if input_ == 'h':
                print('')
                print ("The following products are available in our supermarket:")
                for item in database:
                    print("%s, amount in stock: %s" %(str(item), str(rospy.get_param("/database/"+str(item)+"/amount") )))
                print('')
                print("Currently, the following products are on your shopping list:")
                print(res.list)
                print('')
                input_ = raw_input().strip().lower()
            else:
                res.list.append(input_)
                input_ = raw_input().strip().lower()
        for i in range(2):  print('')

        if len(res.list) > 0 and input_ != 'a':
            res.success = True
            rospy.loginfo('%s: Succeeded' % rospy.get_name())
        else:
            res.success = False
            rospy.loginfo('%s: Failed' % rospy.get_name())

    else:
        rospy.loginfo('%s: Failed' % rospy.get_name())
    print(res)
    return res
