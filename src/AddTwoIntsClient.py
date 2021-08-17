#! /usr/bin/env python

import sys
from roslearning.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse
import rospy

# CONSTANTS
NODE_NAME = "add_two_ints_client"
SERVICE_NAME = "add_two_ints"

# request sum
def request_sum(x, y):
	try:
		add_two_ints = rospy.ServiceProxy(SERVICE_NAME, AddTwoInts)
		resp = add_two_ints(x, y)
		return resp.sum
	except rospy.rosServiceException(e):
		print(f"a service exception occurred: {e}")

# setup client
def setup_client():
	# initialize node
	rospy.init_node(NODE_NAME)
	print(f"Waiting for a node to offer /{SERVICE_NAME} service...")
	rospy.wait_for_service(SERVICE_NAME)
	print(f"Server is awake, can make requests now!")

# Main Driver of this node
def add_two_ints_client():
	if len(sys.argv) == 3:
		x = int(sys.argv[1])
		y = int(sys.argv[2])
	else:
		print(f"{sys.argv[0]} [x, y]")
		sys.exit(1)
	setup_client()
	sum = request_sum(x, y)
	if sum:
		print(f"{x} + {y} = {sum}")
	else:
		print("Failed to get a response!")


if __name__ == "__main__":
	add_two_ints_client()
	
