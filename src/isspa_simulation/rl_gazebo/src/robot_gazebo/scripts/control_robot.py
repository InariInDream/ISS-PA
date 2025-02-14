#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def control_robot():
    rospy.init_node('control_robot', anonymous=True)
    cmd_pub = rospy.Publisher('/hatchback/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    twist = Twist()
    twist.linear.x = 0.5
    twist.angular.z = 0.1

    while not rospy.is_shutdown():
        cmd_pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_robot()
    except rospy.ROSInterruptException:
        pass

