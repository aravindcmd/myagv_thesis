#!/usr/bin/env python
import actionlib
import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time
import cv2
import numpy as np

def camTrack():
    try:
        pass
    except Exception as e:
        raise
    cap = cv2.VideoCapture(0)
    cap.set(3, 160)
    cap.set(4, 120)
    # rospy.init_node('myagv_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while True:
        ret, frame = cap.read()
        low_b = np.uint8([5,5,5])
        high_b = np.uint8([0,0,0])
        mask = cv2.inRange(frame, high_b, low_b)
        contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)[-2:]
        if len(contours) > 0 :
            contours = max(contours, key=cv2.contourArea)
            M = cv2.moments(contours)
            if M["m00"] !=0 :
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("CX : "+str(cx)+"  CY : "+str(cy))
                if cx >= 120 :
                    print("Turn Left")
                    x = 0.1
                    y = 0
                    theta = 0.1
                    publishMovement(x,y,theta)
                if cx < 120 and cx > 40 :
                    print("On Track!")
                    x = 0.1
                    y = 0
                    theta = 0
                    publishMovement(x,y,theta)
                if cx <=40 :
                    print("Turn Right")
                    x = 0.1
                    y = 0
                    theta = -0.1
                    publishMovement(x,y,theta)
                cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
        else :
            print("I don't see the line, stop bot")
            x = 0
            y = 0
            theta = 0
            publishMovement(x,y,theta)
            time.sleep(5)
            cap.release()
            cv2.destroyAllWindows()
        cv2.drawContours(frame, contours, -1, (0,255,0), 1)
        cv2.imshow("Mask",mask)
        cv2.imshow("Frame",frame)
        if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
            print("make the robot stop")
            x = 0
            y = 0
            theta = 0
            publishMovement(x,y,theta)
            cap.release()
            cv2.destroyAllWindows()
            break
        # twist = Twist()
        # twist.linear.x = x;
        # twist.linear.y = y;
        # twist.linear.z = 0
        # twist.angular.x = 0;
        # twist.angular.y = 0;
        # twist.angular.z = theta
        # pub.publish(twist)

    cap.release()
    cv2.destroyAllWindows()
def publishMovement(x,y,theta):
    try:
        print("x , y , thetha recived",x,y,theta)
        # twist = Twist()
        # twist.linear.x = x;
        # twist.linear.y = y;
        # twist.linear.z = 0
        # twist.angular.x = 0;
        # twist.angular.y = 0;
        # twist.angular.z = theta
        # pub.publish(twist)
    except:
        print(e)
    finally:
        print("finllay blocks executed")
        # twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)


def autoNav():
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
            rospy.loginfo("preparing to follow the black line")
            camTrack()
            # return True
            rospy.spin()


    else:
            rospy.loginfo("The robot failed to reach the destination")
            # return False

if __name__ == '__main__':
    try:
        autoNav()
    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
        cap.release()
        cv2.destroyAllWindows()



# goal.target_pose.header.frame_id = 'map'
# goal.target_pose.pose.position.x = new value
# goal.target_pose.pose.position.y = new value
# goal.target_pose.pose.orientation.z = new value
# goal.target_pose.pose.orientation.w = new value
#
# client.send_goal(goal)
# client.wait_for_result()
