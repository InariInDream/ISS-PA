#!/usr/bin/env python3
import gymnasium as gym
from stable_baselines3 import PPO, DQN, DDPG
from stable_baselines3.common.env_checker import check_env
from gymnasium.envs.registration import register
from stable_baselines3.common.logger import configure
import rospy

# 手动注册环境
register(
    id='LineFollowingEnv-v0',
    entry_point='rl_environment:LineFollowingEnv',
)

def main():
    # 初始化 ROS 节点（如果尚未初始化）
    if not rospy.core.is_initialized():
        rospy.init_node('train_rl_agent', anonymous=True)

    # 获取命名空间参数
    robot_namespace = rospy.get_param("~robot_namespace", "robot1")

    # 创建 Gymnasium 环境，并传递命名空间参数
    env = gym.make('LineFollowingEnv-v0', robot_namespace=robot_namespace)

    # 检查环境
    check_env(env)

    # 配置日志
    new_logger = configure(folder="/tmp/gym/", format_strings=["stdout", "csv", "tensorboard"])

    # 创建 RL 模型，强制使用 CPU，调整学习率
    model = DDPG('MlpPolicy', env, verbose=1, device='cpu', learning_rate=3e-4)
    model.set_logger(new_logger)

    # 训练模型，增加训练步数
    model.learn(total_timesteps=500000)

    # 保存模型
    model_path = "/home/inariindream/ppo_line_following"
    model.save(model_path)
    print(f"Model saved to {model_path}")

if __name__ == '__main__':
    main()
