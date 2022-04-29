#!/usr/bin/env python
import os
from socket import *
rospy.init_node("sender")

host = "100.71.173.127" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)

def odom_cb(msg):
    data = bytes(msg, 'utf-8')
    UDPSock.sendto(data, addr)

odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)

