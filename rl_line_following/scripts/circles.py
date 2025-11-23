#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
import threading
from typing import Optional, Dict, Any
from rosgraph_msgs.msg import Clock
import time

class LineFollowingEnv(gym.Env):
    def __init__(self, robot_namespace="robot1"):
        super(LineFollowingEnv, self).__init__()
        if not rospy.core.is_initialized():
            rospy.init_node('rl_line_following_env', anonymous=True)

        self.robot_namespace = robot_namespace

        # Expanded action space for better turn control
        self.action_space = spaces.Box(
            low=np.array([0.5, -8.0]),  # Lower linear speed for turns, wider angular range
            high=np.array([1.5, 1.0]),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=0.0,
            high=10.0,
            shape=(360,),
            dtype=np.float32
        )

        self.laser_sub = rospy.Subscriber(f'/scan', LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher(f'/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(f'/odom', Odometry, self.odom_callback)

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.previous_position = None
        self.cumulative_distance = 0.0
        self.start_time = time.time()
        self.laser_data = np.zeros(360, dtype=np.float32)
        self.laser_received = False
        self.lock = threading.Lock()
        self.reward = 0
        self.done = False
        self.last_reset_time = time.time()
        self.timeout = 60  # Increased timeout for larger track
        self.grace_period = 2
        self.coordinates = []
        self.coord_index = 0
        self.initial_position = None  # To track starting position for lap detection

    def laser_callback(self, data):
        with self.lock:
            self.laser_data = np.array(data.ranges, dtype=np.float32)
            self.laser_data = np.nan_to_num(self.laser_data, nan=10.0, posinf=10.0, neginf=10.0)
            self.laser_received = True

    def odom_callback(self, data):
        with self.lock:
            self.current_linear_speed = data.twist.twist.linear.x
            self.current_angular_speed = data.twist.twist.angular.z
            position = data.pose.pose.position
            current_position = np.array([position.x, position.y, position.z])
            self.coordinates.append(current_position)
            if self.previous_position is not None:
                displacement = current_position - self.previous_position
                distance = np.linalg.norm(displacement)
                self.cumulative_distance += distance
            self.previous_position = current_position

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        twist = Twist()
        twist.linear.x = action[0]
        twist.angular.z = action[1]
        self.cmd_pub.publish(twist)
        rospy.sleep(0.05)

        with self.lock:
            state = self.laser_data.copy()
            current_speed = self.current_linear_speed
            elapsed_time = time.time() - self.last_reset_time
            current_position = self.coordinates[-1] if self.coordinates else np.zeros(3)

        if elapsed_time > 0:
            average_speed = self.cumulative_distance / elapsed_time
        else:
            average_speed = 0.0

        # Reward components
        reward_distance = self.cumulative_distance * 1.0  # Reduced weight
        reward_speed = average_speed * 0.3  # Reduced weight
        min_laser_range = np.min(self.laser_data)

        # Turn reward
        turn_reward = 0.0
        if abs(self.current_angular_speed) > 0.1:  # Turning
            left_ranges = self.laser_data[210:270]  # Left 60°
            right_ranges = self.laser_data[90:150]  # Right 60°
            if np.min(left_ranges) > 0.5 and np.min(right_ranges) > 0.5:
                turn_reward = 0.1

        # Loop completion reward
        reward_loop = 0.0
        if self.initial_position is not None and len(self.coordinates) > 10:
            distance_to_start = np.linalg.norm(current_position[:2] - self.initial_position[:2])
            if distance_to_start < 1.0 and self.cumulative_distance > 10.0:
                reward_loop = 10.0
                self.cumulative_distance = 0.0  # Reset distance after lap

        # Wall proximity penalty
        wall_penalty = -0.5 if min_laser_range < 0.5 else 0.0

        # Comprehensive reward
        reward = reward_distance + reward_speed + turn_reward + reward_loop + wall_penalty

        # Termination conditions
        collision = current_speed < 0.15 or min_laser_range < 0.2
        timeout = elapsed_time > self.timeout
        within_grace_period = elapsed_time < self.grace_period
        terminated = (collision or timeout) and not within_grace_period
        truncated = False
        info = {}

        if terminated:
            self.done = True
            termination_reason = 'Collision' if collision else 'Timeout'
            rospy.loginfo(f"Termination condition met: {termination_reason}")

        # Save coordinates incrementally (optional, retained from original)
        # if len(self.coordinates) > 0:
        #     x_position = self.coordinates[-1][0]
        #     if x_position >= 10:
        #         self.save_coordinates_to_file()

        return state, reward, terminated, truncated, info

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        if seed is not None:
            np.random.seed(seed)

        # Reset Gazebo simulation
        try:
            rospy.wait_for_service('/gazebo/reset_simulation', timeout=5)
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_simulation()
            rospy.loginfo("Gazebo simulation reset.")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s", e)

        # Wait for /clock message
        try:
            rospy.wait_for_message('/clock', Clock, timeout=5)
        except rospy.ROSException:
            rospy.logerr("No /clock message received after reset")

        # Initialize data
        with self.lock:
            self.laser_data = np.zeros(360, dtype=np.float32)
            self.laser_received = False
            self.current_linear_speed = 0.0
            self.previous_position = None
            self.cumulative_distance = 0.0
            self.coordinates = []
            self.initial_position = None  # Reset initial position

        # Set initial position after reset
        rospy.sleep(1)  # Wait for odom to update
        if self.coordinates:
            self.initial_position = self.coordinates[0]

        self.last_reset_time = time.time()
        self.done = False
        rospy.loginfo("Environment reset complete.")

        return self.laser_data, {}

    def save_coordinates_to_file(self):
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
        robot_namespace = "robot1"
        env = LineFollowingEnv(robot_namespace=robot_namespace)
        try:
            while not rospy.is_shutdown():
                observation, _ = env.reset()
                done = False
                while not done:
                    action = env.action_space.sample()
                    observation, reward, done, truncated, info = env.step(action)
        except rospy.ROSInterruptException:
            pass

    main()