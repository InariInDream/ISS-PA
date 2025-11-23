import gym
from gym import spaces
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
import threading

class GazeboCarEnv(gym.Env):
    def __init__(self):
        super(GazeboCarEnv, self).__init__()

        # 定义动作空间（线速度和角速度）
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]),
                                       high=np.array([1.0, 1.0]),
                                       dtype=np.float32)

        # 定义观察空间（激光扫描和速度）
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(360 + 2,), dtype=np.float32)

        # 初始化ROS节点
        rospy.init_node('rl_gazebo_env', anonymous=True)

        # 发布控制指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 订阅传感器数据
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 重置服务客户端
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        # 状态变量
        self.current_scan = np.zeros(360)
        self.current_velocity = np.zeros(2)
        self.done = False
        self.reward = 0.0

        # 锁以保护状态变量
        self.lock = threading.Lock()

    def scan_callback(self, msg):
        with self.lock:
            scan = np.array(msg.ranges[:360])
            # 处理无效值（如inf）
            scan = np.where(np.isinf(scan), 10.0, scan)
            self.current_scan = scan

    def odom_callback(self, msg):
        with self.lock:
            linear = msg.twist.twist.linear.x
            angular = msg.twist.twist.angular.z
            self.current_velocity = np.array([linear, angular])

    def step(self, action):
        # 发送动作到机器人
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self.cmd_pub.publish(twist)

        # 等待仿真更新
        rospy.sleep(0.1)

        # 获取观察
        with self.lock:
            observation = np.concatenate((self.current_scan, self.current_velocity))

        # 计算奖励
        self.reward = self.compute_reward()

        # 判断是否结束
        self.done = self.check_done()

        return observation, self.reward, self.done, {}

    def reset(self):
        # 调用重置服务
        try:
            self.reset_simulation()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # 等待仿真重置
        rospy.sleep(1.0)

        # 等待ROS时间重新开始向前
        self.wait_for_time_forward()

        # 重置状态变量
        with self.lock:
            self.current_scan = np.zeros(360)
            self.current_velocity = np.zeros(2)
            self.done = False
            self.reward = 0.0

        # 获取初始观察
        observation = np.concatenate((self.current_scan, self.current_velocity))
        return observation

    def wait_for_time_forward(self):
        """
        等待ROS时间重新开始向前移动，以避免时间回退导致的异常。
        """
        try:
            clock_msg = rospy.wait_for_message('/clock', Clock, timeout=5)
        except rospy.ROSException as e:
            print("Failed to receive /clock message: %s" % e)
            return

        last_time = clock_msg.clock

        while not rospy.is_shutdown():
            try:
                clock_msg = rospy.wait_for_message('/clock', Clock, timeout=1)
                if clock_msg.clock > last_time:
                    break
                last_time = clock_msg.clock
            except rospy.ROSException:
                print("Waiting for /clock messages...")
                continue

    def compute_reward(self):
        # 示例：根据前方障碍物距离给予奖励
        front_distance = self.current_scan[0]
        if front_distance < 0.5:
            return -1.0  # 碰撞惩罚
        else:
            return 1.0  # 前进奖励

    def check_done(self):
        # 示例：若碰撞则结束
        front_distance = self.current_scan[0]
        if front_distance < 0.5:
            return True
        return False

    def render(self, mode='human'):
        pass

    def close(self):
        pass

def main():
    env = GazeboCarEnv()
    try:
        obs = env.reset()
        for _ in range(10):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            print(f"Action: {action}, Reward: {reward}, Done: {done}")
            if done:
                obs = env.reset()
    finally:
        env.close()

if __name__ == "__main__":
    main()

