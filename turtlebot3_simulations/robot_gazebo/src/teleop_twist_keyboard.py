#!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# from sensor_msgs.msg import Joy

# class TeleopKeyboard:
#     def __init__(self):
#         # 创建 ROS 节点
#         rospy.init_node('teleop_keyboard', anonymous=True)

#         # 控制模型的移动
#         self.pub = rospy.Publisher('/line/command', Twist, queue_size=10)

#         # 设置控制的速率
#         self.rate = rospy.Rate(10)  # 10hz

#         # 初始化速度
#         self.speed = 0.1  # 控制的速度
#         self.move_cmd = Twist()

#         # 获取键盘输入
#         rospy.on_shutdown(self.shutdown)

#         self.control_loop()

#     def control_loop(self):
#         # 监听键盘输入
#         while not rospy.is_shutdown():
#             key = input("Enter WASD to control (W = up, A = left, S = down, D = right): ").strip().lower()
            
#             if key == 'w':
#                 self.move_cmd.linear.z = self.speed  # 向上移动
#             elif key == 's':
#                 self.move_cmd.linear.z = -self.speed  # 向下移动
#             elif key == 'a':
#                 self.move_cmd.linear.x = -self.speed  # 向左移动
#             elif key == 'd':
#                 self.move_cmd.linear.x = self.speed  # 向右移动
#             else:
#                 self.move_cmd.linear.x = 0
#                 self.move_cmd.linear.z = 0

#             self.pub.publish(self.move_cmd)

#             self.rate.sleep()

#     def shutdown(self):
#         rospy.loginfo("Shutting down teleop keyboard control.")
#         self.move_cmd.linear.x = 0
#         self.move_cmd.linear.z = 0
#         self.pub.publish(self.move_cmd)


# if __name__ == '__main__':
#     try:
#         teleop = TeleopKeyboard()
#     except rospy.ROSInterruptException:
#         pass


# import rospy
# from geometry_msgs.msg import Twist

# class RobotControl:
#     def __init__(self):
#         # 初始化ROS节点
#         rospy.init_node('robot_control', anonymous=True)

#         # 创建一个发布者，向/cmd_vel话题发布Twist消息
#         self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#         # 设置发布的频率（例如10Hz）
#         self.rate = rospy.Rate(10)

#     def move_forward(self, speed=0.2):
#         """
#         控制机器人前进
#         :param speed: 线速度，默认值为0.2
#         """
#         move_cmd = Twist()
#         move_cmd.linear.x = speed  # 设置线速度
#         move_cmd.angular.z = 0.0  # 不旋转
#         self.pub.publish(move_cmd)

#     def turn(self, angular_speed=0.5):
#         """
#         控制机器人转向
#         :param angular_speed: 角速度，默认值为0.5
#         """
#         move_cmd = Twist()
#         move_cmd.linear.x = 0.0  # 不前进
#         move_cmd.angular.z = angular_speed  # 设置角速度
#         self.pub.publish(move_cmd)

#     def stop(self):
#         """
#         停止机器人
#         """
#         move_cmd = Twist()
#         move_cmd.linear.x = 0.0
#         move_cmd.angular.z = 0.0
#         self.pub.publish(move_cmd)

#     def control_loop(self):
#         """
#         控制逻辑
#         你可以在这里选择如何控制机器人。
#         """
#         while not rospy.is_shutdown():
#             # 这里可以根据需求调用不同的控制方法
#             self.move_forward(0.2)  # 启动前进
#             self.rate.sleep()
#             rospy.sleep(2)  # 前进2秒
#             self.turn(0.5)  # 启动转向
#             rospy.sleep(2)  # 转向2秒
#             self.stop()  # 停止
#             rospy.sleep(1)  # 停止1秒

# if __name__ == '__main__':
#     try:
#         control = RobotControl()
#         control.control_loop()  # 启动控制循环
#     except rospy.ROSInterruptException:
#         pass
# #

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
