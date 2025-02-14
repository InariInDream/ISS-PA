#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class CmdVelListener:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('cmd_vel_listener', anonymous=True)

        # 使用命名空间发布控制指令（控制 pav_s00 模型）
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 初始化控制小车的速度（线速度和角速度）
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz

    def get_key(self):
        # 用来读取键盘输入
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def control_car(self):
        while not rospy.is_shutdown():
            key = self.get_key()

            # 根据按键更新线速度和角速度
            if key == 'w':  # 前进
                self.linear_speed = 0.5
                self.angular_speed = 0.0
            elif key == 's':  # 后退
                self.linear_speed = -0.5
                self.angular_speed = 0.0
            elif key == 'a':  # 左转
                self.linear_speed = 0.0
                self.angular_speed = 0.5
            elif key == 'd':  # 右转
                self.linear_speed = 0.0
                self.angular_speed = -0.5
            elif key == 'q':  # 退出控制
                break
            else:
                # 如果没有按w, a, s, d，保持当前状态
                self.linear_speed = 0.0
                self.angular_speed = 0.0

            # 创建Twist消息并发布
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.cmd_pub.publish(twist)

            # 打印当前速度
            rospy.loginfo(f"Linear Speed: {self.linear_speed}, Angular Speed: {self.angular_speed}")

            # 控制频率
            self.rate.sleep()

if __name__ == '__main__':
    try:
        listener = CmdVelListener()
        listener.control_car()
    except rospy.ROSInterruptException:
        pass
