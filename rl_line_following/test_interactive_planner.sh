#!/usr/bin/env zsh
# 测试交互式路径规划器

source ~/catkin_ws/devel/setup.zsh

echo "=========================================="
echo "测试交互式路径规划器"
echo "=========================================="
echo ""
echo "1. 检查话题是否存在..."
rostopic list | grep -E "(move_base_simple|planner)"

echo ""
echo "2. 检查服务是否存在..."
rosservice list | grep planner

echo ""
echo "3. 测试清空路径服务..."
rosservice info /planner/clear_path 2>/dev/null || echo "服务尚未启动，请先启动系统"

echo ""
echo "=========================================="
echo "使用说明："
echo "1. 在Rviz中点击工具栏上的 '2D Nav Goal' 工具"
echo "2. 在地图上点击任意位置来添加路径点"
echo "3. 运行 ./clear_path.sh 来清空路径"
echo "=========================================="

