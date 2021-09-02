#! /usr/bin/env python

# run this scritp along with _06Briding_openCV_ROS.py from this package

import rospy, cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# CONSTANTS
NODE_NAME = "image_publisher"
IMAGE_TOPIC = "image"

# GLOBALS
image_pub = None
bridge = CvBridge()

def main():
    rospy.init_node(NODE_NAME)
    global image_pub
    image_pub = rospy.Publisher(IMAGE_TOPIC, Image)

    # attach the webcam's video feed to image_feed object
    image_feed = cv2.VideoCapture(0)

    frame_rate = 1000 // 60
    while not rospy.is_shutdown():
        ret, frame = image_feed.read()

        frame = bridge.cv2_to_imgmsg(frame[:, -1::-1, :], encoding="bgr8")
        # mirroring the frame
        image_pub.publish(frame)


if __name__ == "__main__":
    main()