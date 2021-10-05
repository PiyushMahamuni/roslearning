#! /usr/bin/env/python

# this scripts converts roll-pitch-yaw angles to quaternions
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

print(f"{'-'*50}")
print("Roll-Pitch-Yaw => QUATERNION")
print(f"{'-'*50}")

# Some Random values
roll = math.radians(30)
pitch = math.radians(45)
yaw = math.radians(-35)

# printing these angles
print(f"""Angles in Degrees:
roll: {math.degrees(roll)}
pitch: {math.degrees(pitch)}
yaw: {math.degrees(yaw)}""")

# convert the roll pitch yaw angles to quternion using ROS tf package
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
print(f"{'-'*50}")
print(
    f"The resulting quternion using tf.quaternion_from_euler(): {quaternion}")

# converting quaternion back to euler angles
roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
print(f"{'-'*50}")
print(
    f"Converting the quaternion back to euler angles: {math.degrees(roll)}, {math.degrees(pitch)}, {math.degrees(yaw)}")
