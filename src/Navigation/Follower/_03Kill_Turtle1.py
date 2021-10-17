#! /usr/bin/env python

import rospy
from turtlesim.srv import Kill

# CONSTANTS
NODE_NAME = "turlte1_killer"

def setup():
    rospy.init_node(NODE_NAME)


def main():
    rospy.wait_for_service("/kill")
    killer = rospy.ServiceProxy("/kill", Kill)
    killer("turtle1")


if __name__ == "__main__":
    main()