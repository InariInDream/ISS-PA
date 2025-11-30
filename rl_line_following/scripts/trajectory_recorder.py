#!/usr/bin/env python3
"""
轨迹记录器 - 记录机器人行驶轨迹并支持回放
"""
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse
import yaml
import os
from datetime import datetime

class TrajectoryRecorder:
    def __init__(self):
        rospy.init_node('trajectory_recorder')
        
        # 参数
        self.save_directory = rospy.get_param('~save_directory', 
                                             os.path.expanduser('~/catkin_ws/src/rl_line_following/trajectories'))
        
        # 订阅odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # 发布记录的轨迹
        self.trajectory_pub = rospy.Publisher('/robot/recorded_trajectory', Path, queue_size=10)
        
        # 服务
        self.start_recording_service = rospy.Service('/trajectory/start_recording', Empty, self.start_recording)
        self.stop_recording_service = rospy.Service('/trajectory/stop_recording', Empty, self.stop_recording)
        self.save_trajectory_service = rospy.Service('/trajectory/save', Empty, self.save_trajectory)
        self.load_trajectory_service = rospy.Service('/trajectory/load', Empty, self.load_trajectory)
        
        # 状态
        self.is_recording = False
        self.trajectory = []
        self.loaded_trajectory = None
        
        # 创建保存目录
        os.makedirs(self.save_directory, exist_ok=True)
        
        # 定时器发布轨迹
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_trajectory)
        
        rospy.loginfo("轨迹记录器已启动")
        rospy.loginfo(f"轨迹保存目录: {self.save_directory}")
        rospy.loginfo("服务:")
        rospy.loginfo("  /trajectory/start_recording - 开始记录")
        rospy.loginfo("  /trajectory/stop_recording - 停止记录")
        rospy.loginfo("  /trajectory/save - 保存轨迹")
        rospy.loginfo("  /trajectory/load - 加载轨迹")
    
    def odom_callback(self, msg):
        """记录odom数据"""
        if not self.is_recording:
            return
        
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # 添加时间戳和速度信息
        pose_stamped.header.frame_id = "map"
        self.trajectory.append({
            'timestamp': rospy.Time.now().to_sec(),
            'pose': pose_stamped,
            'velocity': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
        })
    
    def start_recording(self, req):
        """开始记录"""
        self.is_recording = True
        self.trajectory = []
        rospy.loginfo("开始记录轨迹...")
        return EmptyResponse()
    
    def stop_recording(self, req):
        """停止记录"""
        self.is_recording = False
        rospy.loginfo(f"停止记录，已记录 {len(self.trajectory)} 个点")
        return EmptyResponse()
    
    def save_trajectory(self, req):
        """保存轨迹到文件"""
        if len(self.trajectory) == 0:
            rospy.logwarn("没有轨迹可保存")
            return EmptyResponse()
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.save_directory, f"trajectory_{timestamp}.yaml")
        
        # 转换为可序列化格式
        trajectory_data = {
            'timestamp': timestamp,
            'num_points': len(self.trajectory),
            'poses': []
        }
        
        for point in self.trajectory:
            pose_data = {
                'position': {
                    'x': point['pose'].pose.position.x,
                    'y': point['pose'].pose.position.y,
                    'z': point['pose'].pose.position.z
                },
                'orientation': {
                    'x': point['pose'].pose.orientation.x,
                    'y': point['pose'].pose.orientation.y,
                    'z': point['pose'].pose.orientation.z,
                    'w': point['pose'].pose.orientation.w
                },
                'velocity': point['velocity']
            }
            trajectory_data['poses'].append(pose_data)
        
        # 保存到文件
        with open(filename, 'w') as f:
            yaml.dump(trajectory_data, f, default_flow_style=False)
        
        rospy.loginfo(f"轨迹已保存到: {filename}")
        return EmptyResponse()
    
    def load_trajectory(self, req):
        """加载轨迹文件（加载最新的）"""
        # 查找最新的轨迹文件
        trajectory_files = [f for f in os.listdir(self.save_directory) 
                          if f.startswith('trajectory_') and f.endswith('.yaml')]
        
        if not trajectory_files:
            rospy.logwarn("没有找到轨迹文件")
            return EmptyResponse()
        
        # 按时间排序，取最新的
        trajectory_files.sort(reverse=True)
        latest_file = os.path.join(self.save_directory, trajectory_files[0])
        
        # 加载文件
        with open(latest_file, 'r') as f:
            trajectory_data = yaml.safe_load(f)
        
        # 转换为Path消息
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for pose_data in trajectory_data['poses']:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = pose_data['position']['x']
            pose.pose.position.y = pose_data['position']['y']
            pose.pose.position.z = pose_data['position']['z']
            pose.pose.orientation.x = pose_data['orientation']['x']
            pose.pose.orientation.y = pose_data['orientation']['y']
            pose.pose.orientation.z = pose_data['orientation']['z']
            pose.pose.orientation.w = pose_data['orientation']['w']
            path_msg.poses.append(pose)
        
        self.loaded_trajectory = path_msg
        rospy.loginfo(f"已加载轨迹: {latest_file} ({len(path_msg.poses)} 个点)")
        
        return EmptyResponse()
    
    def publish_trajectory(self, event):
        """发布记录的轨迹"""
        if self.loaded_trajectory is not None:
            self.loaded_trajectory.header.stamp = rospy.Time.now()
            self.trajectory_pub.publish(self.loaded_trajectory)

if __name__ == '__main__':
    try:
        recorder = TrajectoryRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

