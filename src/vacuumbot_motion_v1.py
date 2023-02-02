#!/usr/bin/env python

# Title:
#   vacuumbot_motion
#
# Description:
#   This script controls a turtlebot in a Gazebo simulation.
#   The turtlebot moves forward until user stops the program with ctrl-C.
#
# Author:
#   Andreas Jakobsson
#
# Version:
#   1.00 (2023)
#
# Tested with:
#   - Python 3.8.10
#   - Ubuntu 20.04 LTS
#   - ROS Noetic (Turtlebot3)
#

import rospy
from geometry_msgs.msg import Twist

# Advertise publisher for velocity values
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Initialize node
rospy.init_node('motion')

# Set velocity
twist = Twist()
twist.linear.x = 0.5

# Set publishing rate
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	cmd_vel_pub.publish(twist)
	rate.sleep()

twist.linear.x = 0
cmd_vel_pub.publish(twist)
