#!/usr/bin/env python

#This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from math import pi


#Process all the data from the LIDAR
def cb(msg):
    filtered_ranges = [r if r < msg.range_max or r > msg.range_min else msg.range_max for r in msg.ranges]
    min_range = min(filtered_ranges)
    min_angle = filtered_ranges.index(min_range)*360/len(filtered_ranges)
    wall_found = (315 < min_angle  or min_angle < 45) and min_range < 2
    ahead = min(filtered_ranges[:int(len(filtered_ranges)/8)]+filtered_ranges[-1 * int(len(filtered_ranges)/8):])
    s = 1
    #wall ahead and following other wall start turning toward wall ahead
    if ahead < 2 and 250 < min_angle < 290:
        s =1
        min_range = ahead
        min_angle = filtered_ranges.index(min_range)*360/len(filtered_ranges)
    #wall ahead
    elif wall_found:
        s = 1
    #wall to robot's side, following wall
    elif 250 < min_angle < 290:
        s = 2
    pub_state.publish(s)
    pub_distance.publish(min_range)    
    pub_angle.publish(min_angle)
    
    #CALCULATE AND PUBLISH ANYTHING ELSE FOR THE PID



#Init nodeS
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers

pub_state = rospy.Publisher('state', Int16, queue_size = 1)

pub_angle = rospy.Publisher('wall_angle', Float32, queue_size=1)
pub_distance = rospy.Publisher('wall_distance', Float32, queue_size=1)


#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 