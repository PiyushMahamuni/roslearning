#! /usr/bin/env python

from roslearning.srv import AddTwoInts, AddTwoIntsResponse, AddTwoIntsRequest
import rospy
import time

# CONSTANTS
NODE_NAME = "add_two_ints_server"
SERVICE_NAME = "add_two_ints"

# Globals
add_ints_service = None


# Request handling functino for service /add_two_ints
def handle_add_two_ints(req):
	print(f"Returning {req.a} + {req.b} = {req.a + req.b}")
	time.sleep(1)	# sleeping 1 second for dramatic effect of data transfer process
	return AddTwoIntsResponse(req.a + req.b)	# return a response msg

# FUNCTION TO SETUP THE SERVER AND MAKE IT UP AND RUNNING
def setup_server(service):
	#initialize node
	rospy.init_node(NODE_NAME)
	service = rospy.Service(SERVICE_NAME, AddTwoInts, handle_add_two_ints)
	print("Ready to add ints!")

# MAIN DRIVER FUNCTION
def add_two_ints_server():
	setup_server(add_ints_service)
	rospy.spin()


if __name__ == "__main__":
	add_two_ints_server()

