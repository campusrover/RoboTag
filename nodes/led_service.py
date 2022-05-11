#!/usr/bin/env python3

import rospy
# from std_srvs.srv import SetBool
from robotag.srv import Json
import RPi.GPIO as GPIO

YELLOW = 4 #  Yellow
GREEN = 3  # Green
BLUE = 2 # Blue
WHITE = 17 # White
RED = 27 # Red


def set_led_state_callback(req):
    if req.data == "cop":
        GPIO.output(BLUE, True)
        GPIO.output(YELLOW, False)
        GPIO.output(GREEN, False)
        GPIO.output(WHITE, False)
        GPIO.output(RED, False)
    elif req.data == "robber":
        GPIO.output(BLUE, False)
        GPIO.output(YELLOW, False)
        GPIO.output(GREEN, False)
        GPIO.output(WHITE, False)
        GPIO.output(RED, True)
    elif req.data == "robber-user":
        GPIO.output(BLUE, False)
        GPIO.output(YELLOW, False)
        GPIO.output(GREEN, False)
        GPIO.output(WHITE, True)
        GPIO.output(RED, True)
    elif req.data == "cop-user":
        GPIO.output(BLUE, True)
        GPIO.output(YELLOW, False)
        GPIO.output(GREEN, False)
        GPIO.output(WHITE, True)
        GPIO.output(RED, False)
    rospy.loginfo("LED Command Received: %s", req.data)

    return { 'success': req.data,
            'message': 'Successfully changed LED state' }

if __name__ == '__main__':
    rospy.init_node('led_actuator')

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(YELLOW, GPIO.OUT)
    GPIO.setup(GREEN, GPIO.OUT)
    GPIO.setup(BLUE, GPIO.OUT)
    GPIO.setup(WHITE, GPIO.OUT)
    GPIO.setup(RED, GPIO.OUT)
    rospy.Service('set_led_state', Json, set_led_state_callback)
    rospy.loginfo("Service server started. Ready to get requests.")

    rospy.spin()

    GPIO.cleanup()