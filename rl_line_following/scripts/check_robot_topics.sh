#!/usr/bin/env zsh
# 检查机器人话题的脚本

source ~/catkin_ws/devel/setup.zsh

echo "=========================================="
echo "检查机器人话题"
echo "=========================================="
echo ""
echo "第一辆车相关话题:"
rostopic list 2>/dev/null | grep -E "(^/cmd_vel|^/odom|^/scan)" | sort

echo ""
echo "第二辆车相关话题:"
rostopic list 2>/dev/null | grep -E "(robot2|pav_s00)" | sort

echo ""
echo "所有odom话题:"
rostopic list 2>/dev/null | grep odom | sort

echo ""
echo "所有cmd_vel话题:"
rostopic list 2>/dev/null | grep cmd_vel | sort

echo ""
echo "=========================================="

