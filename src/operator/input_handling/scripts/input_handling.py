#!/usr/bin/env python
import rospy
import actionlib
import numpy as np

from custom_msgs.srv import ContinueRequest, ContinueRequestResponse
from custom_msgs.srv import InputRequest, InputRequestResponse
from custom_msgs.srv import UpdateOrder, UpdateOrderResponse

from continue_request import continue_request
from input_request import input_request
from update_order import sort_coordinates

if __name__ == "__main__":
    # Initialize the update order node
    rospy.init_node('input_handling')

    # Initialize the check database /service
    check_database_service = rospy.Service('/update_order/service',UpdateOrder ,sort_coordinates)
    rospy.loginfo('%s is now available' %('/update_order/service'))

    # Initialize the continue request service
    continue_request_service = rospy.Service('continue_request/service',ContinueRequest,continue_request)
    rospy.loginfo('%s is now available' %('/continue_request/service'))

    # Initialize the input_request/service
    check_database_service = rospy.Service('input_request/service',InputRequest ,input_request)
    rospy.loginfo('%s is now available' %('/input_request/service'))

    rospy.spin()
