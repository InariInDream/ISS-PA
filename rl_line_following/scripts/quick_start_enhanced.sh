#!/usr/bin/env zsh
# 快速启动增强版系统

source ~/catkin_ws/devel/setup.zsh

echo "=========================================="
echo "启动增强版ROS路径跟踪系统"
echo "=========================================="
echo ""
echo "新功能："
echo "  ✨ 动态障碍物避障"
echo "  📈 路径平滑优化"
echo "  ⚡ 速度规划（加速度限制）"
echo "  📹 轨迹记录与回放"
echo "  🎯 预测性跟随"
echo "  🎨 可视化增强"
echo ""
echo "提示："
echo "  - 轨迹记录：rosservice call /trajectory/start_recording"
echo "  - 保存轨迹：rosservice call /trajectory/save"
echo "  - 加载轨迹：rosservice call /trajectory/load"
echo ""

roslaunch rl_line_following enhanced_system.launch

