#!/usr/bin/env python

# Title:
#   vacuumbot
#
# Description:
#   This script controls a turtlebot in a Gazebo simulation.
#   The turtlebot drives forward until it detects an obstacle
#   then turns and repeats the cycle.
#
# Author:
#   Andreas Jakobsson
#
# Version:
#   0.1 (2023)
#
# Tested with:
#   - Python 3.8.10
#   - Ubuntu 20.04 LTS
#   - ROS Noetic (Turtlebot3)
#

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

min_collision_distance = 0.5                        # Distance when robot should detect obstacle [m]
loop_freq = 10                                      # Frequency for each publication [Hz]
current_distance = 1.0                              # Variable for current distance sensor value [m]
max_forward_vel = 0.5                               # Forward velocity [m/s]
max_angular_vel = 1                                 # Rotation velocity [rad/s]
driving_forward = True                              # Flag for movement direction

# Subroutine for processing scan detection
def scan_callback(msg):
    global current_distance
    scanarray_length = len(msg.ranges)              # 360 deg for LIDAR
    cone_angle = 20                                 # cone_of_vision/2 [deg]

    range_right = msg.ranges[0:cone_angle:1]
    range_left = msg.ranges[scanarray_length-cone_angle:scanarray_length:1]
    cone_of_vision = range_left + range_right
    current_distance = min(cone_of_vision)

# Advertise subscriber for scan values
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Advertise publisher for velocity values
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Initialize node
rospy.init_node('vacuum')

# Set publishing rate
rate = rospy.Rate(loop_freq)

while not rospy.is_shutdown():
    # Collision detection
    if ( (driving_forward == True) and (current_distance < min_collision_distance) ):
        driving_forward = False
    elif ( (driving_forward == False) and (current_distance >= min_collision_distance) ) :
        driving_forward = True
    
    # Set velocity
    twist = Twist()
    if (driving_forward == True):
        twist.linear.x = max_forward_vel
    else:
        twist.angular.z = max_angular_vel
    
    # Publish velocity
    cmd_vel_pub.publish(twist)

    rate.sleep()