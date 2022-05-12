#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
# from robotag.srv import Json
import RPi.GPIO as GPIO

# YELLOW = 4 #  Yellow
# GREEN = 3  # Green
BLUE = 2 # Blue
# WHITE = 17 # White
# RED = 27 # Red


def blue_callback(req):
    GPIO.output(BLUE, req.data)
    rospy.loginfo("LED Command Received: %s", req.data)
    return { 'success': req.data,
            'message': 'Successfully changed LED state' }

if __name__ == '__main__':
    rospy.init_node('blue_sactuator')
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BLUE, GPIO.OUT)
    rospy.Service('set_blue_led', SetBool, blue_callback)
    rospy.loginfo("Service server started. Ready to get requests.")
    rospy.spin()

    GPIO.cleanup()
