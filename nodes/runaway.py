#!/usr/bin/env python

import rospy
import sys
import math
import tf
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import tf2_ros
import geometry_msgs.msg
import tf_conversions 

# fill in scan callback
def scan_cb(msg):
   global range_ahead
   msg.range_max=5
   msg.range_min=.15
   range_ahead = msg.ranges[0]
   return range_ahead





# odom is also not necessary but very useful
def odom_cb(msg):
   global posex2; global posey2
   global speed2
   posex2= msg.pose.pose.position.x
   posey2=msg.pose.pose.position.y
   speed2=msg.twist.twist.linear.x



# print the state of the robot
def print_state():
   print("---")
   print("STATE: " + state)

   # calculate time since last key stroke
   time_since = rospy.Time.now() - last_key_press_time
   print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

# init node
rospy.init_node('rob2')

# subscribers/publishers
scan_sub = rospy.Subscriber('rob2/scan', LaserScan, scan_cb)


# RUN rosrun prrexamples key_publisher.py to get /keys

odom_sub = rospy.Subscriber('rob2/odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('rob2/cmd_vel', Twist, queue_size=10)


# start in state halted and grab the current time
state = "H"
last_key_press_time = rospy.Time.now()

# set rate
rate = rospy.Rate(10)
#initializes distance measurement
range_ahead=.3
#speed of robots
linear_increment=.2
angular_increment=3.1415/16
#initializes the zigzag angular variable
zig_turn_val=.2
#initializes the time variable
time_start=rospy.Time.now()
#turn multiplier is used in s and z if clauses
turn_mult=-1
#initializes coordinate system and speed
posex2=0
posey2=0
speed2=0
twist=Twist()
# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
   odom=rospy.Publisher('rob2/odom', Odometry, queue_size=10)
   # print out coordinates, speed, the current state and time since last key press
   
   print(posex2)
   print(posey2)
   print(speed2)
   print_state()
  #checks for the closest object
   if(range_ahead<.3):
       twist.linear.x=-.2
       twist.angular.z=.4
   # publish cmd_vel from here 
   cmd_vel_pub = rospy.Publisher('rob2/cmd_vel', Twist, queue_size=10)
   check=(rospy.Time.now().to_sec()-time_start.to_sec())
   if ((check%3)>2):
       x=random.randint(1,2)
       z=random.randint(0,1)
       twist.linear.x=x/10
       twist.angular.z=z/10
   cmd_vel_pub.publish(twist)
   rate.sleep()