#!/usr/bin/env python3
"""
路径平滑器 - 使用样条插值平滑路径点
"""
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

# 尝试导入scipy，如果没有则使用numpy实现
try:
    from scipy.interpolate import interp1d
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    rospy.logwarn("scipy未安装，使用numpy实现平滑（功能可能受限）")

class PathSmoother:
    def __init__(self):
        rospy.init_node('path_smoother')
        
        # 参数
        self.smoothing_factor = rospy.get_param('~smoothing_factor', 0.3)  # 平滑因子（0-1）
        self.interpolation_points = rospy.get_param('~interpolation_points', 5)  # 每两点之间的插值点数
        
        # 订阅原始路径
        self.raw_path_sub = rospy.Subscriber('/planner/path', Path, self.path_callback)
        
        # 发布平滑后的路径
        self.smooth_path_pub = rospy.Publisher('/planner/smooth_path', Path, queue_size=10)
        
        rospy.loginfo("路径平滑器已启动")
    
    def smooth_path(self, path):
        """平滑路径"""
        if len(path.poses) < 2:
            return path
        
        # 提取路径点
        points = []
        for pose in path.poses:
            points.append([pose.pose.position.x, pose.pose.position.y])
        
        points = np.array(points)
        
        # 如果点太少，直接返回
        if len(points) < 3:
            return path
        
        # 使用移动平均平滑
        smoothed_points = self.moving_average_smooth(points, window_size=3)
        
        # 创建平滑后的路径
        smooth_path = Path()
        smooth_path.header = path.header
        smooth_path.header.stamp = rospy.Time.now()
        
        for i, point in enumerate(smoothed_points):
            pose = PoseStamped()
            pose.header = path.header
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            
            # 计算朝向
            if i < len(smoothed_points) - 1:
                next_point = smoothed_points[i + 1]
                dx = next_point[0] - point[0]
                dy = next_point[1] - point[1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
            
            import tf.transformations as tft
            quat = tft.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            smooth_path.poses.append(pose)
        
        return smooth_path
    
    def moving_average_smooth(self, points, window_size=3):
        """移动平均平滑"""
        if len(points) <= window_size:
            return points
        
        smoothed = np.zeros_like(points)
        
        # 处理边界点
        for i in range(window_size // 2):
            smoothed[i] = points[i]
            smoothed[-(i+1)] = points[-(i+1)]
        
        # 处理中间点
        for i in range(window_size // 2, len(points) - window_size // 2):
            window = points[i - window_size // 2:i + window_size // 2 + 1]
            smoothed[i] = np.mean(window, axis=0)
        
        # 混合原始点和平滑点
        result = (1 - self.smoothing_factor) * points + self.smoothing_factor * smoothed
        
        return result
    
    def path_callback(self, msg):
        """处理原始路径"""
        if len(msg.poses) < 2:
            return
        
        # 平滑路径
        smooth_path = self.smooth_path(msg)
        
        # 发布平滑后的路径
        self.smooth_path_pub.publish(smooth_path)
        
        rospy.logdebug(f"路径已平滑: {len(msg.poses)} -> {len(smooth_path.poses)} 个点")

if __name__ == '__main__':
    try:
        smoother = PathSmoother()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

