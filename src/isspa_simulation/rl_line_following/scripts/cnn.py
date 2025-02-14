#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time

class TwoLayerNet(nn.Module):
    def __init__(self, hidden_size=50):
        super(TwoLayerNet, self).__init__()
        self.fc1 = nn.Linear(2, hidden_size)  # 2 inputs (比如激光雷达数据等)
        self.fc2 = nn.Linear(hidden_size, 2)  # 2 outputs (线速度，角速度)
        self.relu = nn.ReLU()

        # 初始化网络权重为零，并调整偏置
        self.fc1.weight = nn.Parameter(torch.zeros_like(self.fc1.weight))
        self.fc1.bias = nn.Parameter(torch.zeros_like(self.fc1.bias))
        self.fc2.weight = nn.Parameter(torch.zeros_like(self.fc2.weight))
        self.fc2.bias = nn.Parameter(torch.tensor([1.0, 1.0]))  # 固定偏置为 1.0

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class LineFollowerController:
    def __init__(self):
        # 初始化 ROS 节点
        if not rospy.core.is_initialized():
            rospy.init_node('line_follower_controller', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.model = TwoLayerNet()  # 使用我们定义的神经网络
        self.model.eval()  # 设置为评估模式

    def run(self):
        # ROS循环控制
        rate = rospy.Rate(10)  # 控制频率10Hz
        
        while not rospy.is_shutdown():
            # 输入固定值(或者根据实际传感器数据来输入)
            # 这里我们可以用固定的输入数据
            input_data = torch.randn(1, 2)  # 随机输入数据 [batch_size=1, input_size=2]
            output = self.model(input_data)  # 神经网络前向传播
            linear_velocity, angular_velocity = output[0].detach().numpy()  # 取得输出，并转为 numpy 数组

            # 固定输出为 [1.0, 1.0] 的场景
            # 线速度和角速度都是 1.0
            # 如果希望是固定的输出，可以直接将以下值硬编码
            linear_velocity = 1.0
            angular_velocity = 0.0

            # 创建 Twist 消息并发布
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity

            # 发布命令
            self.cmd_pub.publish(twist)

            # 控制频率
            rate.sleep()

if __name__ == "__main__":
    try:
        controller = LineFollowerController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
