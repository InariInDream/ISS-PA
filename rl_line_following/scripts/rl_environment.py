#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry  # 导入 Odometry 消息类型
from std_srvs.srv import Empty
import numpy as np
import threading
from typing import Optional, Dict, Any
from rosgraph_msgs.msg import Clock
import time  # 导入 time 模块

class LineFollowingEnv(gym.Env):
    def __init__(self, robot_namespace="robot1"):
        super(LineFollowingEnv, self).__init__()
        if not rospy.core.is_initialized():
            rospy.init_node('rl_line_following_env', anonymous=True)

        # 定义命名空间
        self.robot_namespace = robot_namespace  # 直接使用参数传入的命名空间

        # 定义动作空间：线速度和角速度，分别设定不同的范围
        self.action_space = spaces.Box(
            low=np.array([1, -0.3]),  # 线速度最低1，角速度可以为-2.5
            high=np.array([2, 0.3]),  # 线速度最高2，角速度最高2.5
            dtype=np.float32
        )

        # 定义观测空间：激光雷达数据
        self.observation_space = spaces.Box(
            low=0.0,
            high=10.0,
            shape=(360,),
            dtype=np.float32
        )

        # ROS订阅和发布，使用命名空间
        self.laser_sub = rospy.Subscriber(f'/scan', LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher(f'/cmd_vel', Twist, queue_size=1)

        # 订阅 Odometry 话题以获取小车速度和位置信息
        self.odom_sub = rospy.Subscriber(f'/odom', Odometry, self.odom_callback)
        self.current_linear_speed = 0.0  # 当前线速度
        self.previous_position = None  # 上一个位置
        self.current_angular_speed = 0.0  # 当前角速度

        # 初始化追踪变量
        self.cumulative_distance = 0.0  # 累计行驶距离
        self.start_time = time.time()    # 开始时间

        self.laser_data = np.zeros(360, dtype=np.float32)
        self.laser_received = False
        self.lock = threading.Lock()

        self.reward = 0
        self.done = False

        # 计时器初始化
        self.last_reset_time = time.time()
        self.timeout = 20  # 20秒超时
        self.grace_period = 2  # 2秒缓冲期

        # 初始化小车轨迹保存
        self.coordinates = []
        self.coord_index = 0

    def laser_callback(self, data):
        with self.lock:
            self.laser_data = np.array(data.ranges, dtype=np.float32)
            self.laser_data = np.nan_to_num(self.laser_data, nan=10.0, posinf=10.0, neginf=10.0)
            self.laser_received = True

    def odom_callback(self, data):
        with self.lock:
            # 获取当前线速度
            self.current_linear_speed = data.twist.twist.linear.x
            self.current_angular_speed = data.twist.twist.angular.z

            # 获取当前位置
            position = data.pose.pose.position
            current_position = np.array([position.x, position.y, position.z])

            # 记录小车的坐标
            self.coordinates.append(current_position)

            # 计算位移并更新累计行驶距离
            if self.previous_position is not None:
                displacement = current_position - self.previous_position
                distance = np.linalg.norm(displacement)
                self.cumulative_distance += distance
            self.previous_position = current_position

    def step(self, action):
        # 平滑动作，确保动作在定义的动作空间范围内
        action = np.clip(action, self.action_space.low, self.action_space.high)

        # 发送动作到机器人
        twist = Twist()
        twist.linear.x = action[0]    # 控制线速度（沿 X 轴）
        twist.angular.z = action[1]   # 控制角速度（绕 Z 轴）
        self.cmd_pub.publish(twist)

        # 等待激光数据更新
        rospy.sleep(0.05)  # 调整发布命令的频率

        with self.lock:
            state = self.laser_data.copy()
            current_speed = self.current_linear_speed
            elapsed_time = time.time() - self.last_reset_time

        # 计算平均速度
        if elapsed_time > 0:
            average_speed = self.cumulative_distance / elapsed_time
        else:
            average_speed = 0.0

        # 奖励函数设计
        # 奖励主要基于累计行驶距离和平均速度
        reward_distance = self.cumulative_distance * 1.5      # 奖励累计行驶距离
        reward_speed = average_speed * 0.5                     # 奖励平均速度

        # punishment for collision
        # 碰撞惩罚
        if current_speed < 0.5:
            reward_distance *= 0.5
            reward_speed *= 0.5

        # 综合奖励
        reward = reward_distance + reward_speed

        # 判断是否结束（速度低于0.15 或 超时），且已超过缓冲期
        collision = current_speed < 0.15
        timeout = elapsed_time > self.timeout
        within_grace_period = elapsed_time < self.grace_period

        terminated = (collision or timeout) and not within_grace_period
        truncated = False
        info = {}

        if terminated:
            self.done = True
            termination_reason = 'Collision' if collision else 'Timeout'
            rospy.loginfo(f"Termination condition met: {termination_reason}")
        
        # 增量保存坐标到文件
        # if len(self.coordinates) > 0:
        #     x_position = self.coordinates[-1][0]  # 小车当前的 x 坐标
        #     if x_position >= 10:
        #         self.save_coordinates_to_file()

        # 调试信息
        # rospy.loginfo(f"Action: {action}, Reward: {reward:.3f}, Terminated: {terminated}, "
        #               f"Speed: {current_speed:.2f}, Elapsed Time: {elapsed_time:.2f}s, "
        #               f"Cumulative Distance: {self.cumulative_distance:.2f}m, "
        #               f"Average Speed: {average_speed:.2f}m/s")

        return state, reward, terminated, truncated, info

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        # 设置随机种子
        if seed is not None:
            np.random.seed(seed)

        # 重置 Gazebo 仿真
        try:
            rospy.wait_for_service('/gazebo/reset_simulation', timeout=5)
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_simulation()
            rospy.loginfo("Gazebo simulation reset.")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s", e)
            print("Failed to reset Gazebo simulation.")

        # 等待仿真时间更新，避免时间回退
        try:
            rospy.wait_for_message('/clock', Clock, timeout=5)
            rospy.loginfo("Received /clock message after reset.")
        except rospy.ROSException:
            rospy.logerr("No /clock message received after reset")
            print("No /clock message received after reset.")

        # 初始化观测数据和追踪变量
        with self.lock:
            self.laser_data = np.zeros(360, dtype=np.float32)
            self.laser_received = False
            self.current_linear_speed = 0.0
            self.previous_position = None
            self.cumulative_distance = 0.0
            self.coordinates = []  # 清空坐标列表

        # 重置计时器
        self.last_reset_time = time.time()
        self.done = False
        rospy.loginfo("Environment reset complete.")

        return self.laser_data, {}

    def save_coordinates_to_file(self):
        # 将坐标保存到文件
        file_path = '/home/inariindream/catkin_ws/car_coordinates1.txt'
        with open(file_path, 'a') as f:
            f.write(f"Coord {self.coord_index}:\n")
            for coord in self.coordinates:
                x, y, z = coord[0], coord[1], coord[2]
                linear_speed = self.current_linear_speed
                angular_speed = self.current_angular_speed
                f.write(f"{x}, {y}, {z}, {linear_speed}, {angular_speed}\n")
        rospy.loginfo(f"Coordinates saved to {file_path}")
        self.coord_index += 1

    def render(self, mode='human'):
        pass

    def close(self):
        pass

if __name__ == "__main__":
    def main():
        robot_namespace = "robot1"  # 根据需要调整命名空间
        env = LineFollowingEnv(robot_namespace=robot_namespace)
        # 示例训练逻辑
        try:
            while not rospy.is_shutdown():
                observation, _ = env.reset()
                done = False
                while not done:
                    action = env.action_space.sample()  # 随机动作示例
                    observation, reward, done, truncated, info = env.step(action)
        except rospy.ROSInterruptException:
            pass

    main()
