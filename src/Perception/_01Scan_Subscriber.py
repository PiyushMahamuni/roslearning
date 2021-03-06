#! /usr/bin/env/python
import rospy, math
from sensor_msgs.msg import LaserScan

# CONSTANTS
NODE_NAME = "scan_sub"
SCAN_TOPIC = "scan"

# GLOBALS
# subscirber to SCAN_TOPIC
scan_sub = None

# callback for SCAN_TOPIC
def scan_callback(msg):
    # the msg object will carry a ranges attribute which is a list carrying all the
    # intensities from the range finder

    # discard all the nan values from the ranges.
    msg.ranges = [x for x in msg.ranges if not math.isnan(x)]
    max = min = avg = count = 0
    for ind, val in enumerate(msg.ranges):
        if not math.isnan(val):
            count += 1
            if val > max:
                max = val
                maxInd = ind
            if val < min:
                min = val
                minInd = ind
            avg += val
    if count:
        avg /= count
    else:
        avg = -1
    print(f"Maximum range: {max}, at ind: {maxInd}")
    print(f"Minimum range: {min}, at ind: {minInd}")
    print(f"Average of all ranges: {avg}")


# setup the node
def setup():
    rospy.inint_node(NODE_NAME)

    global scan_sub
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, scan_callback, queue_size=1)


def main():
    setup()
    rospy.spin()


if __name__ == "__main__":
    main()

