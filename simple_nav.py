#!/usr/bin/env python
import actionlib
import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

rospy.init_node('send_client_goal')

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
rospy.loginfo("Waiting for move base server")
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.pose.position.x = -0.276414129138
goal.target_pose.pose.position.y = -0.579892456532
goal.target_pose.pose.orientation.z = 0.727
goal.target_pose.pose.orientation.w = 0.686

client.send_goal(goal)
client.wait_for_result()
rospy.loginfo(client.get_state())
if(client.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        # return True
        rospy.spin()


else:
        rospy.loginfo("The robot failed to reach the destination")
        # return False

# goal.target_pose.header.frame_id = 'map'
# goal.target_pose.pose.position.x = new value
# goal.target_pose.pose.position.y = new value
# goal.target_pose.pose.orientation.z = new value
# goal.target_pose.pose.orientation.w = new value
#
# client.send_goal(goal)
# client.wait_for_result()
