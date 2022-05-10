#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from math import atan2
from tf.transformations import euler_from_quaternion
import random
import os
from socket import *
import json
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped 

import select
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


# TELEOP PART STARTS

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1



def getKey():  
    
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

# TELEOP PART ENDS


# fill in scan callback
def scan_cb(msg):
   global range_ahead
   global range_left
   global range_right
   global range_ahead1
   msg.range_max=5
   msg.range_min=.15
   range_ahead=msg.ranges[0]
   range_ahead1=sum(msg.ranges[350:359]+msg.ranges[0:10])/len(msg.ranges[350:359]+msg.ranges[0:10])
   range_left=msg.ranges[270]
   range_right=msg.ranges[90]
  
   return range_ahead

def get_amcl_coors(msg):
    global posex1, posey1, theta
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion (orientation_list)
    posex1 = msg.pose.pose.position.x
    posey1 = msg.pose.pose.position.y


def amcl_coors_other(msg):
   global posex2; global posey2
   global string
   global data
   print(msg.data[0])
   posex2=msg.data[0]
   posey2=msg.data[1]
   
   
def key_cb(msg):
   global statek; global last_key_press_time
   statek = msg.data
   last_key_press_time = rospy.Time.now()

def left_or_right(angle_to_goal, theta):
            angle_to_goal += math.pi
            theta += math.pi
            goal_range = theta + math.pi
            wrapped = goal_range - (2 * math.pi)
            if abs(angle_to_goal - theta) > 0.2:
                if (goal_range) > (2 * math.pi) and (theta < angle_to_goal or angle_to_goal < wrapped):
                    return("left")
                elif (goal_range) < (2 * math.pi) and (theta < angle_to_goal and angle_to_goal < goal_range):
                    return("left")
                else:
                    return("right")

            else:
                return("straight")
# print the state of the robot
def print_state():
   print("---")
   print("STATE: " + state)

   # calculate time since last key stroke
   time_since = rospy.Time.now() - last_key_press_time
   print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))



name = 'rob1'
state = 'robber'
# init node
rospy.init_node("rob1")
theta = 0.0
# subscribers/publishers
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_amcl_coors)
amcl_sub_other = rospy.Subscriber('/other_odom', Float64MultiArray, amcl_coors_other)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time

last_key_press_time = rospy.Time.now()
first = True
# set rate
rate = rospy.Rate(10)
#initializes distance measurement
range_ahead=0.3
range_left=0.3
range_right=0.3
#initializes the zigzag angular variable

#initializes the time variable
time_start=rospy.Time.now()
#turn multiplier is used in s and z if clauses

#initializes coordinate system and speed
posex1=0
posey1=0
speed1=0
posex2=0
posey2=0
string=""
twist=Twist()
time_switch=rospy.Time.now()
statek='z'
time_select=rospy.Time.now()
# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
    #print(json.loads(string))
    if state == "cop":
        if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
            inc_x = posex2 -posex1
            inc_y = posey2 -posey1
            angle_to_goal = atan2(inc_y, inc_x)
            z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            goal_range = theta + 3.14
            wrapped = goal_range - 6.14
            command = left_or_right(angle_to_goal,theta)
            if (command == "left"):
                print("cop should go left")
                twist.linear.x = 0.1
                twist.angular.z = 0.35
            elif (command == "right"):
                print("cop should go right")
                twist.linear.x = 0.1
                twist.angular.z = -0.35
            elif (command == "straight"):
                print("cop should go straight")
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            # if range_ahead<.3:
            #     inc_x = posex2 -posex1
            #     inc_y = posey2 -posey1
            #     z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            #     if z>range_ahead+.05:
            #         timeis=rospy.Time.now()
            #         while (rospy.Time.now().to_sec()-timeis.to_sec()<3):
            #             twist.linear.x=-.3
            #             twist.angular.z=.6
            #             if z > .05:
            #                 if z < .15:
            #                     twist.linear.x=0
            #                     twist.angular.z=0
            #                     state="robber"
            #                     time_switch=rospy.Time.now()
            if z > .05:
                if z < .30:
                    twist.linear.x=0
                    twist.angular.z=0
                    state="robber"
                    time_switch=rospy.Time.now()
                    first = False
        else:
            twist.linear.x=0
            twist.angular.z=0
    elif state == "robber":
        # if rospy.Time.now().to_sec()-time_select.to_sec()>5:
            
        scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
        print(posex1)
        print(posey1)     
        # publish cmd_vel from here 
        cmd_vel_pub.publish(twist)
        while (rospy.Time.now().to_sec() - time_switch.to_sec() < 4 and first == False):
            twist.angular.z=.4
            twist.linear.x=-.1
            cmd_vel_pub.publish(twist)
        if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
            inc_x = posex2 -posex1
            inc_y = posey2 -posey1
            angle_to_goal = atan2(inc_y, inc_x)
            z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            if z > .05:
                if z < .35:
                    twist.linear.x=0
                    twist.angular.z=0
                    state="cop"
                    time_switch=rospy.Time.now()
        if (range_left<range_right):
            if range_left>.05:
                print("should turn right")
                twist.angular.z=.2
                twist.linear.x=.15
        elif (range_right<range_left):
            if range_right>.05:
                print("should turn left")
                twist.angular.z=-.2
                twist.linear.x=.15
        else:
            twist.angular.z=0
            twist.linear.x=.15
            print("Only go forwards")
        if (range_ahead<.6):
            if range_ahead>0:
                twist.linear.x=0
                twist.angular.z=.2
    elif state=='cop-user':

        #check state change

        if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
            inc_x = posex2 -posex1
            inc_y = posey2 -posey1
            angle_to_goal = atan2(inc_y, inc_x)
            z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            if z > .05:
                if z < .35:
                    twist.linear.x=0
                    twist.angular.z=0
                    state="robber"
                    time_switch=rospy.Time.now()


        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        #rospy.init_node('turtlebot3_teleop')
        cmd_vel_msg = '/cmd_vel'
        cmd_vel_pub = rospy.Publisher(cmd_vel_msg, Twist, queue_size=10)
        turtlebot3_model = rospy.get_param("model", "burger")
        cmd_vel_pub.publish(twist)
        inc_x = posex2 -posex1
        inc_y = posey2 -posey1

        status = 0
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        try:
            print(msg)
            while(1):
                key = getKey()
                if key == 'w' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'x' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'a' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'd' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == ' ' or key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                else:
                    if (key == '\x03'):
                        break

                if status == 20 :
                    print(msg)
                    status = 0

                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                cmd_vel_pub.publish(twist)

        except:
            print(e)
        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
elif state=='robber-user':

        #check state change

        if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
            inc_x = posex2 -posex1
            inc_y = posey2 -posey1
            angle_to_goal = atan2(inc_y, inc_x)
            z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
            if z > .05:
                if z < .3:
                    twist.linear.x=0
                    twist.angular.z=0
                    state="cop"
                    time_switch=rospy.Time.now()


        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        #rospy.init_node('turtlebot3_teleop')
        cmd_vel_msg = '/cmd_vel'
        cmd_vel_pub = rospy.Publisher(cmd_vel_msg, Twist, queue_size=10)
        turtlebot3_model = rospy.get_param("model", "burger")
        cmd_vel_pub.publish(twist)
        inc_x = posex2 -posex1
        inc_y = posey2 -posey1

        status = 0
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        try:
            print(msg)
            while(1):
                key = getKey()
                if key == 'w' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'x' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'a' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == 'd' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                elif key == ' ' or key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                else:
                    if (key == '\x03'):
                        break

                if status == 20 :
                    print(msg)
                    status = 0

                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                cmd_vel_pub.publish(twist)

        except:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            
    # elif state=='cop-user':
   # print out coordinates, speed, the current state and time since last key press
    print(posex2)
    print(posey2)
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    key = getKey()
    if key == 'h': #h for human
        if state == 'robber':
            state = 'robber-user'
        elif state= 'cop:
            state = 'cop-user'
        
    elif key == 'r': #r for robot
        if state == 'robber-user':
            state = 'robber'
        elif state =='cop-user':
            state='cop'
    else:
        print("No change")
    print('Current state: '  + state)
    #checks for the closest object
    cmd_vel_pub.publish(twist)
    rate.sleep()