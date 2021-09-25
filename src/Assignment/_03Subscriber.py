#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# CONSTANTS
IMAGE_TOPIC = "tennis_ball_image"
NODE_NAME = "tennis_ball_detector"

# GLOBALS
bridge = CvBridge()
image_sub = None

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask

def getContours(binary_image):      
    contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        x = int(x)
        y = int(y)
        if (area>100):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cv2.circle(rgb_image, (x,y),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (x,y),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (x,y),5,(150,150,255),-1)
            print ("Area: {}, Perimeter: {}".format(area, perimeter))
    print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)


def image_callback(ros_image):

    # if you look at the documentation of sensor_msgs.msg.Image message, you'll find that all the data of the image
    # is carried as a single dimensional array of unsigned integers, while other fields of the image
    # carry critical enfomation to decode that single dimensional array into appropriate format which are height, width of image
    # and encoding of the image. There are also headers which hold the timestamps and other bits of information.

    print("Recieved an Image")

    try:
        image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # convert to bgr format with unit8 datatype for each channel
    except CvBridgeError as e:
        print(e)
    # from this point onwards, you can work on image object with cv2 library as usual

    yellowLower =(30, 125, 80)
    yellowUpper = (55, 255, 255)

    binary_image_mask = filter_color(image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, image, contours)
    cv2.waitKey(3)


def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    global image_sub
    image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Terminating...")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
