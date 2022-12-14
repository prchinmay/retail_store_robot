#!/usr/bin/env python
import rospy
import actionlib
from custom_msgs.srv import CheckDatabase, CheckDatabaseResponse
from custom_msgs.srv import UpdateDatabase, UpdateDatabaseResponse




def check_database_request(req):
    res = CheckDatabaseResponse()

    # Retreiving the full database
    database = rospy.get_param('/database')


    # checking if all requested products are in the database
    products_in_stock = []
    for product in req.products:
        if product in database:
            res.exist.append(True)
            if products_in_stock.count(product) < rospy.get_param('/database/'+product+'/amount'): 
                res.in_stock.append(True)
                products_in_stock.append(product)
            else:
                res.in_stock.append(False)
        else:
            res.exist.append(False)
            res.in_stock.append(False)
    # if res succes all products are in stock
    if len(res.in_stock) == len(req.products) and len(res.exist) == len(req.products):
        res.success = True
    # if one or multiple items are not in stock return false
    else:
        res.succes = False

    return res


# Updating the database after a product has been picked
def update_database_request(req):
    res = UpdateDatabaseResponse()
    try:
        amount = rospy.get_param('/database/'+req.product+'/amount')
        if amount < 1:
            rospy.logerr('%s is not in stock' %(req.product))
            res.success = False
        else:
            rospy.set_param('/database/'+req.product+'/amount', amount -1)
            res.amount = amount - 1
            res.success = True

    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)
        res.success = False
    print(res)

    return res

if __name__ == "__main__":
    rospy.init_node('database')

    # Check database service, here the request will come in
    check_database_service = rospy.Service('database/check_service',CheckDatabase ,check_database_request)
    rospy.loginfo('%s is now available' %('/database/check_service'))

    # Update database service, after a product has been picked it will be removed from the database.
    update_database_service = rospy.Service('database/update_service',UpdateDatabase ,update_database_request)
    rospy.loginfo('%s is now available' %('/database/update_service'))

    rospy.spin()
