#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

pose= [-1.0, 1.0, 0.0]

if __name__ == '__main__':

    rospy.init_node('robot1')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose.position.x = pose[0]
            goal_pose.target_pose.pose.position.y = pose[1]
            goal_pose.target_pose.pose.position.z = pose[2]
            goal_pose.target_pose.pose.orientation.x = 0.0
            goal_pose.target_pose.pose.orientation.y = 0.0
            goal_pose.target_pose.pose.orientation.z = 0.0
            goal_pose.target_pose.pose.orientation.w = 1.0
            client.send_goal(goal_pose)
            client.wait_for_result()
