#!/usr/bin/env zsh
# 清空路径的快捷脚本

source ~/catkin_ws/devel/setup.zsh

echo "清空路径..."
rosservice call /planner/clear_path
echo "路径已清空！"

