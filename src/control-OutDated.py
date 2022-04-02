#!/usr/bin/env python

#This processes all of the scan values
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from state_definitions import *

# 0 -> 1 if delta distance is less than 1m 
# 1-> 2 if delta distance is less than 10cm (assume 10cm is a touch/caught)
# 2-> 0 if you touched other robot 

#Process all the data from the LIDAR
def cb(msg):

    # Need sub to get the distance between two robots
    distDiff = #dist between robot 1 and 2

    #need to know current state?
    if distDiff < 1: 
        s = 1
    elif distDiff < 0.1:
        s = 2
    else:
        s = 0

    #Determine state
    pub_state.publish(s)
    
    # pub_min_angle.publish(min_angle)
    # pub_min_dist.publish(min_dist)



#Init node
rospy.init_node('control')

#Subscriber for LIDAR
# sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)

#min angle min dist
# pub_min_angle = rospy.Publisher('min_angle', Float32, queue_size = 1)
# pub_min_dist = rospy.Publisher('min_dist', Float32, queue_size = 1)

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 






"""
Dict of robots. {robot name : (state , location)}

sub : location of each robot

pub : where to go 



"""