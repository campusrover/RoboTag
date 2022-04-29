#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from math import cos, pi

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 2.5
#Integral constant
I_CONSTANT = .1
#Derivative constant
D_CONSTANT = .3

#CALLBACKS FOR ANYTHING YOUR PID NEEDS TO SUBSCRIBE TO FROM scan_values_handler
def angle_cb(msg):
    global wall_angle
    wall_angle = msg.data

def distance_cb(msg):
    global wall_distance
    wall_distance = msg.data

def state_cb(msg):
    global state
    state = msg.data


#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('pid_twist', Twist, queue_size = 1)

#SUBSCRIBERS FOR THINGS FROM scan_values_handler YOU MIGHT WANT
angle_sub = rospy.Subscriber("wall_angle", Float32, angle_cb)
distance_sub = rospy.Subscriber("wall_distance", Float32, distance_cb)
state_sub = rospy.Subscriber("state", Int16, state_cb)
#Twist and rate object
t = Twist()
rate = rospy.Rate(10)
state = 0
wall_angle = 0
wall_distance = 0
errors = []
crossed_line = False

while not rospy.is_shutdown():
    #calculate p component
    p_component = 0
    if state == 1:
        #robot turns around if overshoots
        p_component = abs(wall_distance-1) 
    elif state == 2:
        #helps robot turn appropriately to stay on the line
        p_component = -1 * (wall_distance -1) 
    #calculate d component
    #treat derivative as angular distance
    d_component = (270-wall_angle)/270
    errors.append(wall_distance-1)
    while len(errors) > 10:
        errors = errors[1:]
    #calculate i component
    #integral is sum of the last ten errors
    i_component = sum(errors)
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component) 
    t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 