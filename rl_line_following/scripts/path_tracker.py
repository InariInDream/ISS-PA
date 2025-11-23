#!/usr/bin/env python3
"""
路径跟踪控制器 - 让小车按照给定的路径行驶
"""
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
import math
import numpy as np
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage

class PathTracker:
    def __init__(self):
        rospy.init_node('path_tracker')
        
        # 参数
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.2)  # 增加最大线速度
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.5)  # 增加最大角速度
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 0.3)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)  # 增大容差，更容易到达
        self.kp_linear = rospy.get_param('~kp_linear', 1.0)  # 增加线速度增益
        self.kp_angular = rospy.get_param('~kp_angular', 2.0)  # 增加角速度增益
        
        # 路径跟踪模式配置
        # True: 添加新点时，直接前往最新添加的点
        # False: 添加新点时，重新从第一个点开始走完整路径
        self.go_to_newest_point = rospy.get_param('~go_to_newest_point', False)
        
        if self.go_to_newest_point:
            rospy.loginfo("路径跟踪模式: 添加新点时直接前往最新点")
        else:
            rospy.loginfo("路径跟踪模式: 添加新点时重新从第一个点开始")
        
        # 订阅器
        self.path_sub = rospy.Subscriber('/planner/path', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 发布器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 状态
        self.current_path = None
        self.current_pose = None
        self.current_waypoint_idx = 0
        self.path_received = False
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("路径跟踪器已启动")
    
    def path_callback(self, msg):
        """接收新路径"""
        if len(msg.poses) > 0:
            # 检查是否是新增的路径点（路径变长了）
            old_path_length = len(self.current_path.poses) if self.current_path is not None else 0
            new_path_length = len(msg.poses)
            
            self.current_path = msg
            self.path_received = True
            
            # 打印路径点的详细信息
            rospy.loginfo(f"接收到新路径，包含 {len(msg.poses)} 个点：")
            for i, pose in enumerate(msg.poses):
                rospy.loginfo(f"  点 {i+1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
            
            # 根据配置决定行为
            if new_path_length > old_path_length and old_path_length > 0:
                # 检测到新点添加
                if self.go_to_newest_point:
                    # 模式1: 直接前往最新添加的点
                    self.current_waypoint_idx = new_path_length - 1
                    target_point = msg.poses[self.current_waypoint_idx].pose.position
                    rospy.loginfo(f"检测到新路径点，直接前往最新点 #{new_path_length}: ({target_point.x:.2f}, {target_point.y:.2f})")
                else:
                    # 模式2: 重新从第一个点开始走完整路径
                    self.current_waypoint_idx = 0
                    first_point = msg.poses[0].pose.position
                    rospy.loginfo(f"检测到新路径点，重新从第一个点开始: ({first_point.x:.2f}, {first_point.y:.2f})")
                    if self.current_pose is not None:
                        current_x = self.current_pose.position.x
                        current_y = self.current_pose.position.y
                        dist = self.get_distance(current_x, current_y, 
                                                first_point.x, first_point.y)
                        rospy.loginfo(f"  当前距离第一个路径点: {dist:.2f}m")
            else:
                # 全新路径或路径被清空，从第一个点开始
                self.current_waypoint_idx = 0
                if self.current_pose is not None:
                    first_point = msg.poses[0].pose.position
                    current_x = self.current_pose.position.x
                    current_y = self.current_pose.position.y
                    dist = self.get_distance(current_x, current_y, 
                                            first_point.x, first_point.y)
                    rospy.loginfo(f"  当前距离第一个路径点: {dist:.2f}m")
        else:
            rospy.logwarn("接收到空路径")
    
    def odom_callback(self, msg):
        """更新当前位姿"""
        self.current_pose = msg.pose.pose
        # 初始化时打印一次位置信息
        if not hasattr(self, '_odom_first_received'):
            self._odom_first_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            rospy.loginfo(f"收到odom数据，初始位置: ({x:.2f}, {y:.2f})")
    
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
            # 路径已完成
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
        
        # 计算角度误差
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # 计算控制命令
        cmd = Twist()
        
        # 线性速度：基于距离，根据角度误差动态调整
        angle_error_deg = abs(math.degrees(yaw_error))
        
        # 改进的策略：
        # 1. 如果角度误差很大（>90度），先旋转，但允许很小速度前进帮助转向
        # 2. 如果角度误差中等（30-90度），减小速度前进
        # 3. 如果角度误差较小（<30度），正常速度前进
        
        if angle_error_deg > 90.0:
            # 角度误差很大，减小前进速度但不要完全停止
            linear_speed = 0.3 * self.max_linear_speed  # 增加前进速度帮助转向
        elif angle_error_deg > 60.0:
            # 角度误差较大，适度减小速度
            linear_speed = self.kp_linear * min(distance, 3.0) * 0.6
            speed_factor = max(0.3, 1.0 - abs(yaw_error) / (math.pi * 2 / 3))
            linear_speed *= speed_factor
        elif angle_error_deg > 30.0:
            # 角度误差中等，轻微减速
            linear_speed = self.kp_linear * min(distance, 3.0) * 0.8
            speed_factor = max(0.5, 1.0 - abs(yaw_error) / (math.pi / 3))
            linear_speed *= speed_factor
        else:
            # 角度误差较小，正常速度前进
            linear_speed = self.kp_linear * min(distance, 3.0)
            speed_factor = max(0.7, 1.0 - abs(yaw_error) / (math.pi / 4))
            linear_speed *= speed_factor
        
        # 确保最小速度（距离很远时也要有一定速度）
        if distance > 1.0:
            linear_speed = max(linear_speed, 0.2 * self.max_linear_speed)  # 至少20%最大速度
        elif distance > 0.5:
            linear_speed = max(linear_speed, 0.15 * self.max_linear_speed)
        
        cmd.linear.x = max(0.0, min(linear_speed, self.max_linear_speed))
        
        # 角速度：基于角度误差，使用更积极的控制
        # 角度误差越大，角速度越大
        angular_speed = self.kp_angular * yaw_error
        
        # 如果角度误差很大，使用更大的角速度快速转向
        if abs(yaw_error) > math.pi / 2:
            # 使用最大角速度的更大比例
            min_angular = 0.7 * self.max_angular_speed
            if abs(angular_speed) < min_angular:
                angular_speed = math.copysign(min_angular, yaw_error)
        elif abs(yaw_error) > math.pi / 4:
            # 中等角度误差，确保足够的角速度
            min_angular = 0.5 * self.max_angular_speed
            if abs(angular_speed) < min_angular:
                angular_speed = math.copysign(min_angular, yaw_error)
        
        # 确保角速度足够大，不要太小导致转不动
        if abs(angular_speed) < 0.3 and abs(yaw_error) > 0.1:
            angular_speed = math.copysign(0.5, yaw_error)  # 至少0.5 rad/s
        
        cmd.angular.z = max(-self.max_angular_speed, 
                           min(angular_speed, self.max_angular_speed))
        
        # 发布控制命令
        self.cmd_pub.publish(cmd)
        
        # 发布调试信息（每10次打印一次，减少日志刷屏）
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 10 == 0:
            rospy.loginfo(f"[路径跟踪] 当前: ({current_x:.2f}, {current_y:.2f}), "
                         f"目标: ({target_x:.2f}, {target_y:.2f}), "
                         f"距离: {distance:.2f}m, "
                         f"角度误差: {math.degrees(yaw_error):.1f}°, "
                         f"速度: 线={cmd.linear.x:.2f}m/s, 角={cmd.angular.z:.2f}rad/s")
    
    def quaternion_to_yaw(self, orientation):
        """从四元数获取yaw角"""
        import tf.transformations as tft
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tft.euler_from_quaternion(quaternion)
        return euler[2]
    
    def stop(self):
        """停止小车"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        tracker = PathTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

