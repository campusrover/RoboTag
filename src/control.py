#!/usr/bin/env python

#This processes all of the scan values
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from state_definitions import *

# 0 -> 1 if delta distance is less than 1m 
# 1-> 2 if delta distance is less than 10cm (assume 10cm is a touch/caught)
# 2-> 0 if you touched other robot 


"""
robotList : Dict of robots. {robot name : (state , location)}

sub : location of each robot

pub : destinations of each robot

"""

if __name__ == '__main__':
    rospy.init_node('control')


#robot1=cop, robot2=robber => robot1=robber, robot2=cop
def switchRole(robotList, rob1, rob2):
    newlist = robotList
    newlist[rob1] = (0, robotList[rob1][1])
    newlist[rob2] = (2, robotList[rob1][1])
    return newlist


robotList = {}

#add robots to robot list
#Robot 1 starts as the cop
robotList = {robot1:(2:None), robot1:(0:None)}
police = robot1

def cb_location_update(msg):
    robotList[msg.name]=msg.location



move_command = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)


rate = rospy.Rate(10.0)
while not rospy.is_shutdown():

    #update location of each robot 
    for robot in robotList:
        robot[1] =  rospy.Subscriber(#Need to decide if we are using custon msg)
        
    dx = #subscriber to tf 

    #update state 
    if robot[0] != 2 and dx > 1:
        robot[0] == 0
    elif robot[0] != 2 and dx <= 1:
        robot[0] == 1
    elif robot[0] != 2 and dx <= 0.1:
        #touched or caught
        robotList = switchRole(robotList, police, robot) 


    #publish destinations 

    destination = None
    for robot in robotList:
        destination = #Get from tf
        if robot[0] == 0: #wondering
            #
        elif robot[1] == 1: #Running away
            #
        else: #police/chasing
            #
        move_command.publish(destination)
        rate.sleep()


