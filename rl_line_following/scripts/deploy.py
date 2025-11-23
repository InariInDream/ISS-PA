# #!/usr/bin/env python3
# import gym
# import rospy
# from stable_baselines3 import PPO
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from rl_environment import LineFollowingEnv  # 确保模块路径正确
# import numpy as np
# import threading

# class RLController:
#     def __init__(self, model_path, robot_namespace="robot1"):
#         # 初始化 ROS 节点（如果尚未初始化）
#         if not rospy.core.is_initialized():
#             rospy.init_node('rl_controller', anonymous=True)

#         # 加载训练好的模型
#         self.model = PPO.load(model_path)

#         # 创建 Gym 环境
#         self.env = LineFollowingEnv(robot_namespace=robot_namespace)

#         # ROS 发布者
#         self.cmd_pub = rospy.Publisher(f'/cmd_vel', Twist, queue_size=1)

#         # 初始化锁
#         self.lock = threading.Lock()

#         # 订阅 Odometry
#         rospy.Subscriber(f'/odom', Odometry, self.odom_callback)

#         # 等待初始观测
#         rospy.loginfo("等待初始观测...")
#         while not self.env.done and not rospy.is_shutdown():
#             with self.lock:
#                 if self.env.previous_position is not None:
#                     break
#             rospy.sleep(0.1)
#         rospy.loginfo("初始观测已接收，开始控制。")

#     def odom_callback(self, data):
#         rospy.loginfo("Deploy: Received odometry data.")
#         with self.lock:
#             self.env.odom_callback(data)

#     def run(self):
#         rate = rospy.Rate(10)  # 控制频率 10 Hz
#         while not rospy.is_shutdown():
#             with self.lock:
#                 if self.env.done:
#                     rospy.loginfo("环境已终止，正在重置...")
#                     self.env.reset()

#                 # 获取当前观测
#                 observation = np.array([self.env.current_linear_speed, self.env.current_angular_speed], dtype=np.float32)

#             # **填充观测数据至 (360,)**
#             padded_observation = self.pad_observation(observation)

#             # 使用模型预测动作
#             action, _states = self.model.predict(padded_observation, deterministic=True)

#             # 创建 Twist 消息
#             twist = Twist()
#             twist.linear.x = action[0]
#             twist.angular.z = action[1]

#             # 发布动作
#             self.cmd_pub.publish(twist)

#             rate.sleep()

#     def pad_observation(self, observation):
#         """
#         将 (2,) 的观测数据填充到 (360,)。
#         填充值可以是0或其他常数，根据需要调整。
#         """
#         padded = np.zeros(360, dtype=np.float32)
#         padded[:2] = observation
#         # 其余部分保持为0
#         return padded

# if __name__ == "__main__":
#     try:
#         model_path = "/home/inariindream/ppo_line_following.zip"  # 确保路径正确，通常是.zip文件
#         robot_namespace = rospy.get_param("~robot_namespace", "robot1")
#         controller = RLController(model_path=model_path, robot_namespace=robot_namespace)
#         controller.run()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3
import gymnasium as gym
from stable_baselines3 import PPO
from gymnasium.envs.registration import register
import rospy

# 注册环境（根据你的环境调整）
register(
    id='LineFollowingEnv-v0',
    entry_point='rl_environment:LineFollowingEnv',  # 替换为你的环境模块路径
)

def main():
    # 初始化 ROS 节点（如果 Gazebo 环境需要）
    if not rospy.core.is_initialized():
        rospy.init_node('simulate_rl_agent', anonymous=True)

    # 加载训练好的模型
    model_path = "/home/inariindream/circles.zip"  # 替换为你的模型路径
    model = PPO.load(model_path)

    # 创建环境
    env = gym.make('LineFollowingEnv-v0', robot_namespace="robot1")  # 参数与训练时一致

    # 重置环境
    obs, _ = env.reset()

    # 仿真循环
    done = False
    try:
        while not done and not rospy.is_shutdown():
            # 使用模型预测动作
            action, _ = model.predict(obs)
            
            # 执行动作
            obs, reward, done, truncated, info = env.step(action)
            
            # 可选：渲染环境
            env.render()  # 如果环境支持渲染
            
            # 如果环境结束，重置环境
            if done:
                obs, _ = env.reset()
                done = False  # 继续仿真
    except rospy.ROSInterruptException:
        pass

    # 关闭环境
    env.close()

if __name__ == '__main__':
    main()