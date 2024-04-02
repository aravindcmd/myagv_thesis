import cv2
import numpy as np
#import RPi.GPIO as GPIO
cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)
rospy.init_node('myagv_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
while True:
    ret, frame = cap.read()
    low_b = np.uint8([5,5,5])
    high_b = np.uint8([0,0,0])
    mask = cv2.inRange(frame, high_b, low_b)
    contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
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
            if cx < 120 and cx > 40 :
                print("On Track!")
                x = 0.1
                y = 0
                theta = 0
            if cx <=40 :
                print("Turn Right")
                x = 0.1
                y = 0
                theta = -0.1
            cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
    else :
        print("I don't see the line, stop bot")
        x = 0
        y = 0
        theta = 0
    cv2.drawContours(frame, contours, -1, (0,255,0), 1)
    cv2.imshow("Mask",mask)
    cv2.imshow("Frame",frame)
    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
        print("make the robot stop")
        x = 0
        y = 0
        theta = 0
        break
    twist = Twist()
    twist.linear.x = x;
    twist.linear.y = y;
    twist.linear.z = 0
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = theta
    pub.publish(twist)
    
cap.release()
cv2.destroyAllWindows()
