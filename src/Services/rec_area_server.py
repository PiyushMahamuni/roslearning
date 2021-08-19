#! /usr/bin/env python

import rospy
from roslearning.srv import rec_area, rec_areaResponse

# CONSTANTS
NODE_NAME = "rec_area_client_py"
SERVICE_NAME = "rec_area"

# GLOBALS
service = None

# request handler
def rec_area_handler(req):
    rospy.loginfo(f"Request: {req.width}, {req.height}")
    rospy.loginfo(f"Returning Response: {req.width * req.height}")
    return rec_areaResponse(req.width * req.height)


# setup node
def setup():
    # initialize node
    rospy.init_node(NODE_NAME)
    # initialize service
    service = rospy.Service(SERVICE_NAME, rec_area, rec_area_handler)
    rospy.loginfo("Ready to serve!")


if __name__ == "__main__":
    setup()
    rospy.spin()    # prevent program from terminating unless killed by user