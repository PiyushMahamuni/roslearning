#! /usr/bin/env python

# run this script along with _01streaming_video.py from this package

import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# CONSTANTS
IMAGE_TOPIC = "image"

# GLOBALS
bridge = CvBridge()
image_sub = None


def image_callback(ros_image):

    # if you look at the documentation of sensor_msgs.msg.Image message, you'll find that all the data of the image
    # is carried as a single dimensional array of unsigned integers, while other fields of the image
    # carry critical enfomation to decode that single dimensional array into appropriate format which are height, width of image
    # and encoding of the image. There are also headers which hold the timestamps and other bits of information.

    print("Recieved an Image")

    try:
        other_image = image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # convert to bgr format with unit8 datatype for each channel
    except CvBridgeError as e:
        print(e)
    # from this point onwards, you can work on image object with cv2 library as usual
    rows, columns, channels = image.shape
    if columns > 200 and rows > 200:
        cv2.circle(image, (100, 100), 90, 255, -1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image, "Webcam Activated with ROS and OpenCV!",
                (5, 350), font, 0.7, (255, 0, 255), 2, cv2.LINE_AA)
    # paras- image, text, starting point, font, font size, pixel color, thickness, NO IDEA what the last para is
    
    cv2.imshow("Image Window", image)
    cv2.waitKey(3)


def main(args):
    rospy.init_node("image_converter", anonymous=True)
    # for turtlebot3 waffle
    # image topic = "/camera/rgb/image_raw/compressed"
    # for usb cam
    # image topic = "/usb_cam/image_raw"
    # for the webcam image topic = "/usb_cam/image_raw"
    global image_sub
    image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Terminating...")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
