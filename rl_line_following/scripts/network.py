#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
import time
from threading import Lock
from sklearn.preprocessing import StandardScaler

# 定义神经网络模型 (加载预训练模型)
class SimpleFCNN(nn.Module):
    def __init__(self):
        super(SimpleFCNN, self).__init__()
        self.fc1 = nn.Linear(2, 64)  # 输入层 (x, y)
        self.fc2 = nn.Linear(64, 128)  # 隐藏层
        self.fc3 = nn.Linear(128, 2)  # 输出层，2个输出：线速度和角速度

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x


# 加载训练好的模型
def load_model(model_path='car_navigation_fcnn.pth'):
    model = SimpleFCNN()
    model.load_state_dict(torch.load(model_path))
    model.eval()  # 设置为评估模式
    return model


# 使用训练好的模型进行预测
def predict_velocity(model, x, y, scaler_X, scaler_y):
    # 标准化输入
    scaled_input = scaler_X.transform([[x, y]])  # 使用训练时的scaler进行转换
    scaled_input_tensor = torch.tensor(scaled_input, dtype=torch.float32)
    
    # 预测
    with torch.no_grad():
        scaled_output = model(scaled_input_tensor)
        # 反标准化输出
        prediction = scaler_y.inverse_transform(scaled_output.numpy())
        return prediction[0]


# 控制小车行为的类
class LineFollowerController:
    def __init__(self):
        # 初始化 ROS 节点
        if not rospy.core.is_initialized():
            rospy.init_node('line_follower_controller', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_data = np.zeros(360, dtype=np.float32)
        self.laser_received = False
        self.lock = Lock()

        # 加载训练好的模型
        self.model = load_model('/home/inariindream/catkin_ws/src/rl_line_following/scripts/car_navigation_fcnn.pth')

        # 获取激光雷达数据
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # 获取小车的位置信息
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_position = np.zeros(3)
        self.previous_position = np.zeros(3)

        # 初始化
        self.cumulative_distance = 0.0
        self.previous_time = time.time()

        # 加载标准化器 (假设你已经在训练时保存了它们)
        # 如果你没有保存它们，可以在这里手动定义或者从文件加载
        self.scaler_X = StandardScaler()
        self.scaler_y = StandardScaler()

        # 训练数据（假设训练数据已经存在，并且可以用来拟合 scaler_X 和 scaler_y）
        # 你可以直接在这里使用你的训练数据进行拟合
        # 这里假设你有训练数据 X_train 和 y_train
        X_train = np.array([[0, 0], [1, 1], [2, 2]])  # 替换为实际的训练数据
        y_train = np.array([[0, 0], [1, 1], [2, 2]])  # 替换为实际的训练标签

        # 对 X 和 y 进行标准化
        self.scaler_X.fit(X_train)
        self.scaler_y.fit(y_train)

    def laser_callback(self, data):
        with self.lock:
            self.laser_data = np.array(data.ranges, dtype=np.float32)
            self.laser_data = np.nan_to_num(self.laser_data, nan=10.0, posinf=10.0, neginf=10.0)
            self.laser_received = True

    def odom_callback(self, data):
        position = data.pose.pose.position
        self.current_position = np.array([position.x, position.y, position.z])

        # 计算累计行驶的距离
        displacement = self.current_position - self.previous_position
        distance = np.linalg.norm(displacement)
        self.cumulative_distance += distance

        self.previous_position = self.current_position

    def control_car(self):
        rate = rospy.Rate(10)  # 控制频率10Hz
        
        while not rospy.is_shutdown():
            # 获取当前的坐标
            x, y = self.current_position[0], self.current_position[1]
            
            # 使用模型预测线速度和角速度
            v_pred, omega_pred = predict_velocity(self.model, x, y, self.scaler_X, self.scaler_y)

            # 创建 Twist 消息并发布
            twist = Twist()
            twist.linear.x = v_pred
            twist.angular.z = omega_pred

            # 发布消息
            self.cmd_pub.publish(twist)

            rate.sleep()


if __name__ == "__main__":
    try:
        controller = LineFollowerController()
        controller.control_car()
    except rospy.ROSInterruptException:
        pass
