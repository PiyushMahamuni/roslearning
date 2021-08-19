#! /usr/bin/env python

import rospy
from roslearning.srv import rec_area
import sys

# CONSTANTS
NODE_NAME = "rec_are_client_py"
SERVICE_NAME = "rec_area"

# GLOBALS
service_proxy = None

# setup node
def setup():
    global service_proxy
    # initialize node
    rospy.init_node(NODE_NAME)
    # initialize client
    service_proxy = rospy.ServiceProxy(SERVICE_NAME, rec_area)


# function to make a request through client object
def req_area(width, height):
    # wait for a node to offer the req service
    rospy.wait_for_service(SERVICE_NAME)
    try:
        resp = service_proxy(width, height)
        return resp.area
    except rospy.ServiceException:
        rospy.loginfo(F"Failed to get response")

# main driver
def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} {{width}} {{height}}")
        sys.exit(1)
    
    setup()
    print(f"area: {req_area(float(sys.argv[1]), float(sys.argv[2]))}")


if __name__ == "__main__":
    main()