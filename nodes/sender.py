#!/usr/bin/env python
import os
from socket import *
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

rospy.init_node("sender")

host = "100.71.173.127" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)

def odom_cb(msg):
    if msg is not None:
        data = bytes(str(msg), 'utf-8')
        UDPSock.sendto(data, addr)

odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)

while not rospy.is_shutdown():
    hi = "r"