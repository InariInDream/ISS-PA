#!/usr/bin/env python3
"""
增强版路径跟踪控制器 - 带动态障碍物避障功能
"""
import rospy
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import tf.transformations as tft

class EnhancedPathTracker:
    def __init__(self):
        rospy.init_node('enhanced_path_tracker')
        
        # 参数
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.2)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)
        self.kp_linear = rospy.get_param('~kp_linear', 1.0)
        self.kp_angular = rospy.get_param('~kp_angular', 2.0)
        
        # 障碍物避障参数
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)  # 障碍物检测距离（米）
        self.avoidance_angle = rospy.get_param('~avoidance_angle', 30.0)  # 避障角度范围（度）
        self.avoidance_speed_reduction = rospy.get_param('~avoidance_speed_reduction', 0.5)  # 遇到障碍物时的速度降低比例
        
        # 速度规划参数
        self.max_acceleration = rospy.get_param('~max_acceleration', 0.5)  # 最大加速度 m/s^2
        self.max_deceleration = rospy.get_param('~max_deceleration', 0.8)  # 最大减速度 m/s^2
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # 订阅器
        self.path_sub = rospy.Subscriber('/planner/path', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # 发布器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 状态
        self.current_path = None
        self.current_pose = None
        self.current_waypoint_idx = 0
        self.path_received = False
        self.laser_data = None
        
        # 轨迹记录
        self.trajectory_history = []
        self.trajectory_pub = rospy.Publisher('/robot/trajectory', Path, queue_size=10)
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        self.trajectory_timer = rospy.Timer(rospy.Duration(0.5), self.publish_trajectory)
        
        rospy.loginfo("增强版路径跟踪器已启动（带障碍物避障）")
    
    def path_callback(self, msg):
        """接收新路径"""
        if len(msg.poses) > 0:
            old_path_length = len(self.current_path.poses) if self.current_path is not None else 0
            new_path_length = len(msg.poses)
            
            self.current_path = msg
            self.path_received = True
            
            # 路径跟踪模式配置
            self.go_to_newest_point = rospy.get_param('~go_to_newest_point', False)
            
            if new_path_length > old_path_length and old_path_length > 0:
                if self.go_to_newest_point:
                    self.current_waypoint_idx = new_path_length - 1
                else:
                    self.current_waypoint_idx = 0
            else:
                self.current_waypoint_idx = 0
            
            rospy.loginfo(f"接收到新路径，包含 {len(msg.poses)} 个点")
    
    def odom_callback(self, msg):
        """更新当前位姿"""
        self.current_pose = msg.pose.pose
        
        # 记录轨迹
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.trajectory_history.append(pose_stamped)
        
        # 限制历史长度
        if len(self.trajectory_history) > 1000:
            self.trajectory_history.pop(0)
    
    def laser_callback(self, msg):
        """接收激光雷达数据"""
        self.laser_data = msg
    
    def check_obstacle_ahead(self, current_yaw, lookahead_distance=1.0):
        """检查前方是否有障碍物"""
        if self.laser_data is None:
            return False, 0.0
        
        # 获取前方角度范围内的最小距离
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        ranges = np.array(self.laser_data.ranges)
        
        # 过滤无效值
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) == 0:
            return False, 0.0
        
        # 检查前方±avoidance_angle度范围内的障碍物
        avoidance_angle_rad = math.radians(self.avoidance_angle)
        num_ranges = len(ranges)
        
        # 找到前方角度对应的索引
        front_idx = num_ranges // 2  # 假设激光雷达前方是中间
        
        # 检查前方扇形区域
        start_idx = max(0, int(front_idx - avoidance_angle_rad / angle_increment))
        end_idx = min(num_ranges, int(front_idx + avoidance_angle_rad / angle_increment))
        
        front_ranges = ranges[start_idx:end_idx]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        
        if len(front_ranges) == 0:
            return False, 0.0
        
        min_distance = np.min(front_ranges)
        
        # 如果最小距离小于阈值，有障碍物
        if min_distance < self.obstacle_threshold:
            return True, min_distance
        
        return False, min_distance
    
    def get_distance(self, x1, y1, x2, y2):
        """计算两点之间的距离"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def normalize_angle(self, angle):
        """将角度归一化到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def quaternion_to_yaw(self, orientation):
        """从四元数获取yaw角"""
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tft.euler_from_quaternion(quaternion)
        return euler[2]
    
    def apply_speed_planning(self, target_speed, dt=0.1):
        """应用速度规划（加速度限制）"""
        # 计算速度变化
        speed_diff = target_speed - self.current_linear_speed
        
        if speed_diff > 0:
            # 加速
            max_change = self.max_acceleration * dt
            speed_change = min(speed_diff, max_change)
        else:
            # 减速
            max_change = self.max_deceleration * dt
            speed_change = max(speed_diff, -max_change)
        
        self.current_linear_speed += speed_change
        return self.current_linear_speed
    
    def control_loop(self, event):
        """主控制循环"""
        if not self.path_received or self.current_path is None:
            return
        
        if self.current_pose is None:
            return
        
        # 获取当前位置
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # 检查是否到达目标点
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.stop()
            return
        
        # 获取当前目标点
        target_pose = self.current_path.poses[self.current_waypoint_idx]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # 计算距离目标点的距离
        distance = self.get_distance(current_x, current_y, target_x, target_y)
        
        # 如果接近目标点，移动到下一个点
        if distance < self.goal_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path.poses):
                rospy.loginfo("已到达路径终点")
                self.stop()
                return
            target_pose = self.current_path.poses[self.current_waypoint_idx]
            target_x = target_pose.pose.position.x
            target_y = target_pose.pose.position.y
        
        # 计算目标方向
        target_yaw = math.atan2(target_y - current_y, target_x - current_x)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # 检查前方障碍物
        has_obstacle, obstacle_distance = self.check_obstacle_ahead(current_yaw)
        
        # 计算控制命令
        cmd = Twist()
        
        # 线性速度计算
        angle_error_deg = abs(math.degrees(yaw_error))
        
        if angle_error_deg > 90.0:
            linear_speed = 0.3 * self.max_linear_speed
        elif angle_error_deg > 60.0:
            linear_speed = self.kp_linear * min(distance, 3.0) * 0.6
            speed_factor = max(0.3, 1.0 - abs(yaw_error) / (math.pi * 2 / 3))
            linear_speed *= speed_factor
        elif angle_error_deg > 30.0:
            linear_speed = self.kp_linear * min(distance, 3.0) * 0.8
            speed_factor = max(0.5, 1.0 - abs(yaw_error) / (math.pi / 3))
            linear_speed *= speed_factor
        else:
            linear_speed = self.kp_linear * min(distance, 3.0)
            speed_factor = max(0.7, 1.0 - abs(yaw_error) / (math.pi / 4))
            linear_speed *= speed_factor
        
        # 障碍物避障：如果有障碍物，降低速度
        if has_obstacle:
            linear_speed *= self.avoidance_speed_reduction
            rospy.logwarn(f"检测到前方障碍物！距离: {obstacle_distance:.2f}m，降低速度")
        
        # 应用速度规划
        linear_speed = self.apply_speed_planning(linear_speed)
        
        if distance > 1.0:
            linear_speed = max(linear_speed, 0.2 * self.max_linear_speed)
        elif distance > 0.5:
            linear_speed = max(linear_speed, 0.15 * self.max_linear_speed)
        
        cmd.linear.x = max(0.0, min(linear_speed, self.max_linear_speed))
        
        # 角速度计算
        angular_speed = self.kp_angular * yaw_error
        
        if abs(yaw_error) > math.pi / 2:
            min_angular = 0.7 * self.max_angular_speed
            if abs(angular_speed) < min_angular:
                angular_speed = math.copysign(min_angular, yaw_error)
        
        if abs(angular_speed) < 0.3 and abs(yaw_error) > 0.1:
            angular_speed = math.copysign(0.5, yaw_error)
        
        cmd.angular.z = max(-self.max_angular_speed, 
                           min(angular_speed, self.max_angular_speed))
        
        # 发布控制命令
        self.cmd_pub.publish(cmd)
    
    def publish_trajectory(self, event):
        """发布轨迹历史"""
        if len(self.trajectory_history) < 2:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.trajectory_history[-100:]  # 只发布最近100个点
        
        self.trajectory_pub.publish(path_msg)
    
    def stop(self):
        """停止小车"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

if __name__ == '__main__':
    try:
        tracker = EnhancedPathTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

