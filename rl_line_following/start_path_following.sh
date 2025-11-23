#!/usr/bin/env zsh

# Rviz路径跟踪和摄像头查看启动脚本

echo "=========================================="
echo "启动Rviz路径跟踪和摄像头查看功能"
echo "=========================================="

# 检查是否在catkin工作空间中
if [ ! -f ~/catkin_ws/devel/setup.bash ]; then
    echo "警告: 未找到 catkin 工作空间，请先编译:"
    echo "cd ~/catkin_ws && catkin_make"
    exit 1
fi

# 设置环境变量
source ~/catkin_ws/devel/setup.zsh

# 启动主launch文件
echo ""
echo "正在启动节点..."
echo "这将启动:"
echo "  - Gazebo仿真"
echo "  - Rviz可视化"
echo "  - 路径规划器"
echo "  - 路径跟踪器"
echo "  - 摄像头查看器"
echo ""
echo "提示:"
echo "  - 摄像头图像会在OpenCV窗口中显示"
echo "  - 在Rviz中可以查看路径和机器人模型"
echo "  - 小车会自动按照预定义的路径行驶"
echo ""

roslaunch rl_line_following rviz_path_following.launch

