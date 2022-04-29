#!/usr/bin/env python
import os
from socket import *
host = "100.74.41.103"
port = 13000
buf = 1024
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
UDPSock.bind(addr)
while True:
    (data, addr) = UDPSock.recvfrom(buf)
    data=data.decode("utf-8") 
    print ("Received message: " + data)
    if data == "exit":
        break
UDPSock.close()
os._exit(0)