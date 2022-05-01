#!/usr/bin/env python
import rospy
import math 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = 0.0
pitch = 0.0
yaw = 0.0

def get_rotation (msg):
    global roll, pitch, yaw 
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, 'Yaw =',math.degrees(yaw))

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, get_rotation) # geometry_msgs/PoseWithCovariance pose

r = rospy.Rate(1) 
while not rospy.is_shutdown():
    quat = quaternion_from_euler (roll, pitch,yaw) 
    print(quat) 
    r.sleep()
