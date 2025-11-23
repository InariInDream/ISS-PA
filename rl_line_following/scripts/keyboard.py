#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import rospy
# from geometry_msgs.msg import Twist
# import sys, select, termios, tty

# # 显示控制说明 (保持不变)
# msg = """
# Control Your Robot!
# ---------------------------
# Moving around:
#         w
#    a    s    d
#         x

# w/s : move forward/backward
# a/d : turn left/right
# x or space : force stop

# CTRL-C to quit
# """

# # 定义按键绑定 (保持不变)
# moveBindings = {
#         'w':(1,0),
#         'a':(0,1),
#         'd':(0,-1),
#         's':(-1,0),
#     }

# # 定义速度参数
# speed = 0.5  # 线速度
# turn = 0.3   # 角速度

# def getKey():
#     """从终端读取单个按键，非阻塞"""
#     # 注意：这里不再恢复终端设置
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # 0.1秒超时
#     if rlist:
#         key = sys.stdin.read(1)
#     else:
#         key = ''
#     return key

# def vels(speed, turn):
#     """打印当前速度"""
#     return "currently:\tspeed %s\tturn %s " % (speed, turn)

# if __name__=="__main__":
#     # 1. 在程序开始时，保存终端的原始设置
#     settings = termios.tcgetattr(sys.stdin)

#     rospy.init_node('teleop_keyboard_controller')
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#     # 2. 创建一个Rate对象，保证循环以10Hz的频率运行
#     rate = rospy.Rate(10)

#     # 初始化速度变量
#     target_speed = 0
#     target_turn = 0
#     control_speed = 0
#     control_turn = 0

#     try:
#         print(msg)
#         print(vels(speed, turn))
        
#         # while not rospy.is_shutdown() 是更标准的ROS循环方式
#         while not rospy.is_shutdown():
#             key = getKey()
            
#             # 根据按键更新目标速度
#             if key in moveBindings.keys():
#                 target_speed = moveBindings[key][0] * speed
#                 target_turn = moveBindings[key][1] * turn
#             elif key == ' ' or key == 'x':
#                 target_speed = 0
#                 target_turn = 0
#             # 3. 改进停止逻辑：如果超时没有按键，则目标速度为0
#             elif key == '':
#                 target_speed = 0
#                 target_turn = 0
#             # 退出
#             elif (key == '\x03'): # CTRL-C
#                 break
            
#             # 为了平滑，我们可以简单地直接使用目标速度，或者保留平滑逻辑
#             # 这里我们为了简单和响应及时，直接赋值
#             control_speed = target_speed
#             control_turn = target_turn

#             # 创建并发布Twist消息
#             twist = Twist()
#             twist.linear.x = control_speed
#             twist.linear.y = 0
#             twist.linear.z = 0
#             twist.angular.x = 0
#             twist.angular.y = 0
#             twist.angular.z = control_turn
#             pub.publish(twist)
            
#             # 4. 使用rate.sleep()来控制循环频率
#             rate.sleep()

#     except Exception as e:
#         print(e)

#     finally:
#         # 程序退出前发布一条空速度指令，让机器人停下来
#         twist = Twist()
#         twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
#         twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
#         pub.publish(twist)

#         # 5. 在程序结束时，恢复终端的原始设置
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



import rospy
from geometry_msgs.msg import Twist

def move_forward():
    """
    一个简单的ROS节点，用于发布速度指令，让机器人沿X轴直线前进。
    """
    # 1. 初始化ROS节点
    rospy.init_node('test_move_controller', anonymous=True)

    # 2. 创建一个发布器，发布到/cmd_vel话题
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 3. 设置循环频率 (例如 10 Hz)
    rate = rospy.Rate(10)

    # 4. 创建一个Twist消息，并设置前进的线速度
    move_cmd = Twist()
    # --- 修改这里来改变速度 ---
    move_cmd.linear.x = 0.5  # 沿X轴前进的速度 (米/秒)
    move_cmd.angular.z = 0   # 角速度为0，不旋转

    # 定义一个在关闭节点时执行的函数
    def shutdown_hook():
        rospy.loginfo("节点关闭，发送停止命令...")
        stop_cmd = Twist() # 默认所有速度都为0
        cmd_pub.publish(stop_cmd)

    # 注册关闭处理函数，当按下CTRL+C时，会调用此函数
    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("节点启动，开始让小车沿X轴前进...")
    rospy.loginfo(f"发布速度: linear.x = {move_cmd.linear.x}")

    # 5. 循环发布速度指令，直到节点被关闭
    while not rospy.is_shutdown():
        cmd_pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass