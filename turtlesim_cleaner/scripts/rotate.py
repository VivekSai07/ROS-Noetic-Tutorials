#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

PI = math.pi

def rotate():
    rospy.init_node('robot_cleaner', anonymous=True)
    vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's rotate your robot: ")
    speed = int(input("Input your speed (degrees/sec): "))
    angle = int(input("Type your distance (degrees): "))
    clockwise = int(input("Clockwise (0/1) ?: "))

    angular_speed = (speed * 2 * PI ) / 360
    relative_angle = (angle * 2 * PI) / 360

    if clockwise:
        vel_msg.angular.z = - abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    while not rospy.is_shutdown():    
        t0 = rospy.Time.now().to_sec()
        curr_angle = 0
        
        while curr_angle < relative_angle:
            vel_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            curr_angle = angular_speed * (t1 - t0)
        
        vel_msg.angular.z = 0
        vel_publisher.publish(vel_msg)
        rotate()
        rospy.spin()

if __name__ == "__main__":
    try:
        rotate()
    except rospy.ROSInterruptException():
        pass