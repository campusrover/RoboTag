#!/usr/bin/env python
import os
from socket import *
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
rospy.init_node("receiver")
pub = rospy.Publisher('/other_odom', Odometry, queue_size = 1)

host = "100.71.173.127"
port = 13000
buf = 1024
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
UDPSock.bind(addr)
while True:
    (data, addr) = UDPSock.recvfrom(buf)
    data=data.decode("utf-8")
    pub.publish(data)
    if data == "exit":
        break
UDPSock.close()
os._exit(0)