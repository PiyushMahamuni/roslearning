#! /usr/bin/env python

import rospy, cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# CONSTANTS
NODE_NAME = "tennis_ball_publisher"
IMAGE_TOPIC = "tennis_ball_image"

# GLOBALS
image_pub = None
bridge = CvBridge()

def main():
    rospy.init_node(NODE_NAME)
    global image_pub
    image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1)

    # attach video file
    image_feed = cv2.VideoCapture("/home/phineas/Video/tennis-ball-video.mp4")

    frame_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, frame = image_feed.read()

        try:
            frame = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        image_pub.publish(frame)
        frame_rate.sleep()
    
    image_feed.release()


if __name__ == "__main__":
    main()