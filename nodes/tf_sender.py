#!/usr/bin/env python
import os
from socket import *
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# Because of transformations
import tf_conversions 
import tf2_ros
import geometry_msgs.msg
import math 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node("sender")
#100.71.173.127
#100.74.41.103
host = "100.71.173.127" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)



roll = 0.0
pitch = 0.0
yaw = 0.0

def get_rotation (msg):
    if msg is not None:
        global roll, pitch, yaw 
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, 'Yaw =',math.degrees(yaw))
        mess=str(msg.pose.pose.position.x)+" "+str(msg.pose.pose.position.y)
        data = bytes(str(mess), 'utf-8')
        UDPSock.sendto(data, addr)
    

sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, get_rotation) # geometry_msgs/PoseWithCovariance pose

while not rospy.is_shutdown():
    hi = "r"


