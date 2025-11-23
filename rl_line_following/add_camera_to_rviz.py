#!/usr/bin/env python3
"""
在Rviz中添加摄像头显示的命令行帮助脚本
"""
import rospy
import subprocess
import time

def show_rviz_camera_info():
    print("=" * 60)
    print("在Rviz中显示摄像头画面的方法")
    print("=" * 60)
    print()
    print("方法1：使用Rviz GUI添加摄像头显示")
    print("----------------------------------------")
    print("1. 在Rviz中点击 'Add' 按钮")
    print("2. 选择 'By topic' 选项卡")
    print("3. 找到 'depth_camera' 分组")
    print("4. 选择 'rgb/image_raw' (Image类型)")
    print("   或者 'depth/image_raw' (Image类型)")
    print("5. 点击 'OK'")
    print()
    print("方法2：使用命令行启动带摄像头的Rviz")
    print("----------------------------------------")
    print("运行以下命令启动带摄像头显示的节点：")
    print()
    print("rosrun image_view image_view image:=/depth_camera/rgb/image_raw")
    print()
    print("或者深度图像：")
    print()
    print("rosrun image_view image_view image:=/depth_camera/depth/image_raw")
    print()
    print("方法3：使用OpenCV窗口（推荐）")
    print("----------------------------------------")
    print("运行以下命令启动OpenCV图像查看器：")
    print()
    print("python3 scripts/camera_viewer.py")
    print()
    print("=" * 60)

if __name__ == '__main__':
    show_rviz_camera_info()

