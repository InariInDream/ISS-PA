#!/usr/bin/env python3
"""
跟随机器人控制器 - 让第二辆车跟随第一辆车的轨迹
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math
import tf.transformations as tft

class FollowerRobot:
    def __init__(self):
        rospy.init_node('follower_robot')
        
        # 参数
        self.follow_distance = rospy.get_param('~follow_distance', 1.0)  # 跟随距离（米）
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.2)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.5)
        self.kp_linear = rospy.get_param('~kp_linear', 1.0)
        self.kp_angular = rospy.get_param('~kp_angular', 2.0)
        
        # 话题名称配置
        leader_odom_topic = rospy.get_param('~leader_odom_topic', '/odom')
        follower_odom_topic = rospy.get_param('~follower_odom_topic', '/robot2/odom')
        follower_cmd_topic = rospy.get_param('~follower_cmd_topic', '/robot2/cmd_vel')
        
        # 订阅第一辆车的位置
        self.leader_odom_sub = rospy.Subscriber(leader_odom_topic, Odometry, self.leader_odom_callback)
        
        # 订阅第二辆车的位置
        self.follower_odom_sub = rospy.Subscriber(follower_odom_topic, Odometry, self.follower_odom_callback)
        
        # 发布第二辆车的控制命令
        self.cmd_pub = rospy.Publisher(follower_cmd_topic, Twist, queue_size=10)
        
        rospy.loginfo(f"第一辆车odom话题: {leader_odom_topic}")
        rospy.loginfo(f"第二辆车odom话题: {follower_odom_topic}")
        rospy.loginfo(f"第二辆车cmd_vel话题: {follower_cmd_topic}")
        
        # 状态
        self.leader_pose = None
        self.follower_pose = None
        self.leader_velocity = None
        
        # 历史轨迹（用于更平滑的跟随）
        self.leader_trajectory = []
        self.trajectory_max_size = 50
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("跟随机器人控制器已启动")
        rospy.loginfo(f"跟随距离: {self.follow_distance}m")
    
    def leader_odom_callback(self, msg):
        """接收第一辆车的位置"""
        self.leader_pose = msg.pose.pose
        self.leader_velocity = msg.twist.twist
        
        # 记录轨迹
        point = Point()
        point.x = msg.pose.pose.position.x
        point.y = msg.pose.pose.position.y
        point.z = msg.pose.pose.position.z
        self.leader_trajectory.append(point)
        
        # 限制轨迹长度
        if len(self.leader_trajectory) > self.trajectory_max_size:
            self.leader_trajectory.pop(0)
    
    def follower_odom_callback(self, msg):
        """接收第二辆车的位置"""
        self.follower_pose = msg.pose.pose
    
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
        if self.leader_pose is None or self.follower_pose is None:
            return
        
        # 获取第一辆车的位置和朝向
        leader_x = self.leader_pose.position.x
        leader_y = self.leader_pose.position.y
        leader_yaw = self.quaternion_to_yaw(self.leader_pose.orientation)
        
        # 获取第二辆车的位置和朝向
        follower_x = self.follower_pose.position.x
        follower_y = self.follower_pose.position.y
        follower_yaw = self.quaternion_to_yaw(self.follower_pose.orientation)
        
        # 预测性跟随：预测第一辆车未来的位置
        # 根据第一辆车的速度和角速度预测未来位置
        prediction_time = 0.5  # 预测时间（秒）
        
        if self.leader_velocity is not None:
            leader_vx = self.leader_velocity.linear.x
            leader_vz = self.leader_velocity.angular.z
            
            # 预测未来位置（考虑速度和角速度）
            predicted_leader_yaw = leader_yaw + leader_vz * prediction_time
            predicted_leader_x = leader_x + leader_vx * math.cos(leader_yaw) * prediction_time
            predicted_leader_y = leader_y + leader_vx * math.sin(leader_yaw) * prediction_time
            
            # 目标位置在预测位置的后方
            target_x = predicted_leader_x - self.follow_distance * math.cos(predicted_leader_yaw)
            target_y = predicted_leader_y - self.follow_distance * math.sin(predicted_leader_yaw)
        else:
            # 如果没有速度信息，使用当前位置
            target_x = leader_x - self.follow_distance * math.cos(leader_yaw)
            target_y = leader_y - self.follow_distance * math.sin(leader_yaw)
        
        # 计算到目标位置的距离
        distance = self.get_distance(follower_x, follower_y, target_x, target_y)
        
        # 计算目标方向
        target_yaw = math.atan2(target_y - follower_y, target_x - follower_x)
        
        # 计算角度误差
        yaw_error = self.normalize_angle(target_yaw - follower_yaw)
        
        # 计算控制命令
        cmd = Twist()
        
        # 线性速度：基于距离和第一辆车的速度
        angle_error_deg = abs(math.degrees(yaw_error))
        
        # 如果角度误差很大，先转到位
        if angle_error_deg > 45.0:
            linear_speed = 0.2 * self.max_linear_speed
        else:
            # 根据距离和第一辆车的速度计算
            if self.leader_velocity is not None:
                leader_speed = self.leader_velocity.linear.x
                # 跟随速度应该略小于或等于第一辆车的速度
                linear_speed = min(leader_speed * 0.9, self.max_linear_speed)
            else:
                linear_speed = self.kp_linear * min(distance, 2.0)
            
            # 根据角度误差调整速度
            speed_factor = max(0.5, 1.0 - abs(yaw_error) / (math.pi / 3))
            linear_speed *= speed_factor
        
        cmd.linear.x = max(0.0, min(linear_speed, self.max_linear_speed))
        
        # 角速度：基于角度误差
        angular_speed = self.kp_angular * yaw_error
        
        # 如果角度误差很大，确保足够的角速度
        if abs(yaw_error) > math.pi / 2:
            min_angular = 0.7 * self.max_angular_speed
            if abs(angular_speed) < min_angular:
                angular_speed = math.copysign(min_angular, yaw_error)
        
        cmd.angular.z = max(-self.max_angular_speed, 
                           min(angular_speed, self.max_angular_speed))
        
        # 发布控制命令
        self.cmd_pub.publish(cmd)
        
        # 调试信息（每10次打印一次）
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 10 == 0:
            rospy.loginfo(f"[跟随] 第一辆车: ({leader_x:.2f}, {leader_y:.2f}), "
                         f"第二辆车: ({follower_x:.2f}, {follower_y:.2f}), "
                         f"目标: ({target_x:.2f}, {target_y:.2f}), "
                         f"距离: {distance:.2f}m, "
                         f"角度误差: {math.degrees(yaw_error):.1f}°")

if __name__ == '__main__':
    try:
        follower = FollowerRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

