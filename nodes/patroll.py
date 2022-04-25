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

# fill in scan callback
def scan_cb(msg):
   global range_ahead
   msg.range_max=5
   msg.range_min=.15
   range_ahead=msg.ranges[0]
   return range_ahead




# odom is also not necessary but very useful
def odom_cb(msg):
   global posex1; global posey1
   global speed1
   global rot_q1
   global theta
   posex1= msg.pose.pose.position.x
   posey1=msg.pose.pose.position.y
   speed1=msg.twist.twist.linear.x
   rot_q1 = msg.pose.pose.orientation
   (roll, pitch, theta) = euler_from_quaternion([rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w])

def odom_cb2(msg):
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
rospy.init_node('rob1')
theta = 0.0
# subscribers/publishers
scan_sub = rospy.Subscriber('rob1/scan', LaserScan, scan_cb)


# RUN rosrun prrexamples key_publisher.py to get /keys

odom_sub = rospy.Subscriber('rob1/odom', Odometry, odom_cb)
odom_sub2 = rospy.Subscriber('rob2/odom', Odometry, odom_cb2)
cmd_vel_pub = rospy.Publisher('rob1/cmd_vel', Twist, queue_size=10)

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
posex1=0
posey1=0
speed1=0
posex2=0
posey2=0
speed2=0
twist=Twist()
# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
   inc_x = posex2 -posex1
   inc_y = posey2 -posey1
   angle_to_goal = atan2(inc_y, inc_x)
   if (angle_to_goal - theta) > 0.1:
     twist.linear.x = 0.1
     twist.angular.z = 0.3
   elif (angle_to_goal - theta) < -0.1:
     twist.linear.x = 0.1
     twist.angular.z = -0.3
   else:
     twist.linear.x = 0.25
     twist.angular.z = 0.0
   odom=rospy.Publisher('rob1/odom', Odometry, queue_size=10)
   # print out coordinates, speed, the current state and time since last key press
   print(posex2)
   print(posey2)
   print(speed2)
   print_state()
  #checks for the closest object
   cmd_vel_pub.publish(twist)
   rate.sleep()