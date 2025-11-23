#!/usr/bin/env python3
"""
调试脚本：检查路径跟踪器的状态
"""
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

def path_callback(msg):
    print(f"\n=== 路径信息 ===")
    print(f"路径包含 {len(msg.poses)} 个点：")
    for i, pose in enumerate(msg.poses):
        print(f"  点 {i+1}: ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})")
    print(f"Frame: {msg.header.frame_id}")

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print(f"\n=== Odom信息 ===")
    print(f"位置: ({x:.3f}, {y:.3f}, {z:.3f})")
    print(f"Frame: {msg.header.frame_id}")
    print(f"Child Frame: {msg.child_frame_id}")

if __name__ == '__main__':
    rospy.init_node('debug_path_tracker')
    
    rospy.Subscriber('/planner/path', Path, path_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    print("调试脚本已启动，监听路径和odom信息...")
    print("按Ctrl+C停止\n")
    
    rospy.spin()

