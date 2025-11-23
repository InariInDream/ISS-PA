#!/usr/bin/env python3

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from math import pi, sin, cos

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)
        
        # 创建move_base动作客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.client.wait_for_server()
        rospy.loginfo("连接到move_base服务器")
        
        # 预定义路径点（可根据实际地图调整）
        self.waypoints = [
            [(-1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],  # [x, y, z], [x, y, z, w]
            [(2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(2.0, 1.0, 0.0), (0.0, 0.0, 0.7071, 0.7071)],
            [(1.0, 1.0, 0.0), (0.0, 0.0, 1.0, 0.0)],
            [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
        ]
        
        # 开始路径跟踪
        self.follow_path()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    
    def create_goal(self, position, orientation):
        """创建导航目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        
        # 设置方向
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        return goal
    
    def follow_path(self):
        """按顺序跟踪路径点"""
        for i, waypoint in enumerate(self.waypoints):
            position, orientation = waypoint
            
            # 创建导航目标
            goal = self.create_goal(position, orientation)
            
            rospy.loginfo(f"导航到路径点 {i+1}/{len(self.waypoints)}: 位置 ({position[0]}, {position[1]})")
            
            # 发送目标并等待结果
            self.client.send_goal(goal)
            wait_result = self.client.wait_for_result()
            
            if not wait_result:
                rospy.logerr("动作服务器无响应")
                return False
            
            # 检查目标状态
            goal_state = self.client.get_state()
            if goal_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"到达路径点 {i+1}")
            else:
                rospy.logwarn(f"无法到达路径点 {i+1}, 状态: {goal_state}")
                
        rospy.loginfo("路径跟踪完成")
        return True

if __name__ == '__main__':
    try:
        PathFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("路径跟踪被中断")