#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('robot_cleaner', anonymous=True)
    vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's move your robot: ")
    speed = int(input("Input your speed: "))
    distance = int(input("Type your distance: "))
    isForward = int(input("Forward (0/1) ?: "))

    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = - abs(speed)
    
    while not rospy.is_shutdown():

        t0 = rospy.Time.now().to_sec()
        curr_dist = 0

        while curr_dist < distance:
            vel_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            curr_dist = speed * (t1 - t0)
            # print(f'Current Distance: {curr_dist}')
        
        vel_msg.linear.x = 0
        vel_publisher.publish(vel_msg)
        # move()
        rospy.spin()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass