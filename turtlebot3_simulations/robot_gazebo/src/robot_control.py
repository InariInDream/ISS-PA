#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class RobotControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('robot_control', anonymous=True)

        # 创建一个发布者，向/cmd_vel话题发布Twist消息
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 设置发布的频率（例如10Hz）
        self.rate = rospy.Rate(10)

    def move_forward(self, speed=0.2):
        """
        控制机器人前进
        :param speed: 线速度，默认值为0.2
        """
        move_cmd = Twist()
        move_cmd.linear.x = speed  # 设置线速度
        move_cmd.angular.z = 0.0  # 不旋转
        self.pub.publish(move_cmd)

    def turn(self, angular_speed=0.5):
        """
        控制机器人转向
        :param angular_speed: 角速度，默认值为0.5
        """
        move_cmd = Twist()
        move_cmd.linear.x = 0.0  # 不前进
        move_cmd.angular.z = angular_speed  # 设置角速度
        self.pub.publish(move_cmd)

    def stop(self):
        """
        停止机器人
        """
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.pub.publish(move_cmd)

    def control_loop(self):
        """
        控制逻辑
        你可以在这里选择如何控制机器人。
        """
        while not rospy.is_shutdown():
            # 这里可以根据需求调用不同的控制方法
            self.move_forward(0.2)  # 启动前进
            self.rate.sleep()
            rospy.sleep(2)  # 前进2秒
            self.turn(0.5)  # 启动转向
            rospy.sleep(2)  # 转向2秒
            self.stop()  # 停止
            rospy.sleep(1)  # 停止1秒

if __name__ == '__main__':
    try:
        control = RobotControl()
        control.control_loop()  # 启动控制循环
    except rospy.ROSInterruptException:
        pass
