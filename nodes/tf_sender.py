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

rospy.init_node("sender")
# 100.71.173.127
host = "100.74.41.103" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)

def odom_cb(msg):
    if msg is not None:
        data = bytes(str(msg), 'utf-8')
        UDPSock.sendto(data, addr)

odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)


# def handle_pose(msg):
#     br = tf2_ros.TransformBroadcaster()
#     t = geometry_msgs.msg.TransformStamped()
#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = "world"
#     t.child_frame_id = "Donatello"
#     t.transform.translation.x = msg.x
#     t.transform.translation.y = msg.y
#     t.transform.translation.z = 0.0
#     q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
#     t.transform.rotation.x = q[0]
#     t.transform.rotation.y = q[1]
#     t.transform.rotation.z = q[2]
#     t.transform.rotation.w = q[3]

#     br.sendTransform(t)
def GetCurrentRobotPose(self,frame="map"):
        self.tfl.waitForTransform(frame, "base_link", rospy.Time(), rospy.Duration(1.0))
        (trans,rot) = self.tfl.lookupTransform(frame, "base_link", rospy.Time(0))
        
        """
        Remove all the rotation components except yaw
        """
        euler = tf.transformations.euler_from_quaternion(rot)
        rot = tf.transformations.quaternion_from_euler(0,0,euler[2])    
    
        current_pose = PoseWithCovarianceStamped()
        current_pose.header.stamp = rospy.get_rostime()
        current_pose.header.frame_id = frame
        current_pose.pose.pose = Pose(Point(trans[0], trans[1], 0.0), Quaternion(rot[0],rot[1],rot[2],rot[3]))
        
        return current_pose 


if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    rospy.Subscriber('/pose',
                     handle_pose)
    rospy.spin()

while not rospy.is_shutdown():
    hi = "r"


