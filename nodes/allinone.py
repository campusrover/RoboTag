#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from math import atan2
from tf.transformations import euler_from_quaternion
import random
import os
from socket import *
import json
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped 

# fill in scan callback
def scan_cb(msg):
   global range_ahead
   global range_left
   global range_right
   msg.range_max=5
   msg.range_min=.15
   range_ahead=msg.ranges[0]
   range_left=msg.ranges[270]
   range_right=msg.ranges[90]
  
   return range_ahead

def get_amcl_coors(msg):
    global posex1, posey1, theta
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion (orientation_list)
    posex1 = msg.pose.pose.position.x
    posey1 = msg.pose.pose.position.y


def amcl_coors_other(msg):
   global posex2; global posey2
   global string
   global data
   print(msg.data[0])
   posex2=msg.data[0]
   posey2=msg.data[1]
   
   
def key_cb(msg):
   global statek; global last_key_press_time
   statek = msg.data
   last_key_press_time = rospy.Time.now()

# print the state of the robot
def print_state():
   print("---")
   print("STATE: " + state)

   # calculate time since last key stroke
   time_since = rospy.Time.now() - last_key_press_time
   print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))


count=0
name = 'rob1'
state = 'robber'
# init node
rospy.init_node("rob1")
theta = 0.0
# subscribers/publishers
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_amcl_coors)
amcl_sub_other = rospy.Subscriber('/other_odom', Float64MultiArray, amcl_coors_other)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time

last_key_press_time = rospy.Time.now()

# set rate
rate = rospy.Rate(10)
#initializes distance measurement
range_ahead=0.3
range_left=0.3
range_right=0.3
#initializes the zigzag angular variable

#initializes the time variable
time_start=rospy.Time.now()
#turn multiplier is used in s and z if clauses

#initializes coordinate system and speed
posex1=0
posey1=0
speed1=0
posex2=0
posey2=0
string=""
twist=Twist()
time_switch=rospy.Time.now()
statek='z'

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
    #print(json.loads(string))
    if state == "cop":
        if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
            if statek=='c':
                state='cop-user'
            inc_x = posex2 -posex1
            inc_y = posey2 -posey1
            angle_to_goal = atan2(inc_y, inc_x)
            print("ANGLE @ GOAL")
            print(angle_to_goal)
            print(theta)
            z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            goal_range = theta + 3.14
            wrapped = goal_range - 6.14
            if abs(angle_to_goal - theta) > 0.1:
                if (goal_range) > 6.14 and (theta < angle_to_goal or angle_to_goal < wrapped):
                    twist.linear.x = 0.0
                    twist.angular.z = -0.3
                elif (goal_range) < 6.14 and (theta < angle_to_goal or angle_to_goal < goal_range):
                    twist.linear.x = 0.0
                    twist.angular.z = -0.3
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3

            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            # if range_ahead<.3:
            #     inc_x = posex2 -posex1
            #     inc_y = posey2 -posey1
            #     z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            #     if z>range_ahead+.05:
            #         timeis=rospy.Time.now()
            #         while (rospy.Time.now().to_sec()-timeis.to_sec()<3):
            #             twist.linear.x=-.3
            #             twist.angular.z=.6
            #             if z > .05:
            #                 if z < .15:
            #                     twist.linear.x=0
            #                     twist.angular.z=0
            #                     state="robber"
            #                     time_switch=rospy.Time.now()
            if z > .05:
                if z < .3:
                    twist.linear.x=0
                    twist.angular.z=0
                    state="robber"
                    time_switch=rospy.Time.now()
    elif state == "robber":
        if statek=='r':
                state='robber-user'
        scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
        print(posex1)
        print(posey1)
        
            
   # publish cmd_vel from here 
        cmd_vel_msg = '/cmd_vel'
        cmd_vel_pub = rospy.Publisher(cmd_vel_msg, Twist, queue_size=10)
        check=(rospy.Time.now().to_sec()-time_start.to_sec())
        cmd_vel_pub.publish(twist)
        if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
            inc_x = posex2 -posex1
            inc_y = posey2 -posey1
            angle_to_goal = atan2(inc_y, inc_x)
            z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            if z > .05:
                if z < .3:
                    twist.linear.x=0
                    twist.angular.z=0
                    state="cop"
                    time_switch=rospy.Time.now()
        if (range_left<range_right):
            if range_left>.05:
                print("should turn right")
                twist.angular.z=.2
                twist.linear.x=.1
        elif (range_right<range_left):
            if range_right>.05:
                print("should turn left")
                twist.angular.z=-.2
                twist.linear.x=.1
        else:
            twist.angular.z=0
            twist.linear.x=.1
            print("Only go forwards")
        if (range_ahead<.6):
            if range_ahead>0:
                twist.linear.x=0
                twist.angular.z=.2
    elif state=='robber-user':
        temp = "temps"
    # elif state=='cop-user':
   # print out coordinates, speed, the current state and time since last key press
    print(posex2)
    print(posey2)
  
    print_state()

    #checks for the closest object
    cmd_vel_pub.publish(twist)
    rate.sleep()