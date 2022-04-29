#!/usr/bin/env python
import os
from socket import *
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
from std_msgs.msg import Float64MultiArray




rospy.init_node("receiver")
mypub = rospy.Publisher('/other_odom', Float64MultiArray,queue_size = 10)


host = "100.74.41.103"
port = 13000
buf = 1024
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
UDPSock.bind(addr)
while not rospy.is_shutdown():
    (data, addr) = UDPSock.recvfrom(buf)
    data=data.decode('utf-8')
    data=data.split("\n")
    x=data[10]
    x=x.split()
    x=x[1]
    y=data[11]
    y=y.split()
    y=y[1]
    x=float(x)
    y=float(y)
    my_msg = Float64MultiArray()
    d=[x, y, 67.654236]
    
    my_msg.data = d
    mypub.publish(my_msg)
    if data == "exit":
        break
UDPSock.close()
os._exit(0)