#!/usr/bin/env python

# Title:
#   vacuumbot_sensor
#
# Description:
#   This script publishes distance sensor measurements
#   of a turtlebot in a Gazebo simulation.
#   Meant to be used for obstacle detection.
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
from sensor_msgs.msg import LaserScan

# Subroutine for processing scan detection.
# First element of the scan is straight in front
# of the robot.
def scan_callback(msg):
    current_distance = msg.ranges[0]
    print("Distance to obstacle: %0.1f m" % current_distance)

# Initialize node
rospy.init_node('current_distance')

# Subscribe to scan topic
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

rospy.spin()
