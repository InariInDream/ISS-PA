#!/usr/bin/env zsh

# 检查摄像头话题的脚本

echo "=========================================="
echo "检查可用的摄像头话题"
echo "=========================================="

# 列出所有可能包含camera的话题
echo ""
echo "扫描摄像头相关话题..."
echo ""

CAMERA_TOPICS=$(rostopic list 2>/dev/null | grep -E "(camera|rgb|depth|image)")

if [ -z "$CAMERA_TOPICS" ]; then
    echo "未找到摄像头话题。"
    echo ""
    echo "可能的原因:"
    echo "  1. Gazebo还未启动"
    echo "  2. 机器人模型中没有配置摄像头"
    echo "  3. 摄像头话题名称不同"
    echo ""
    echo "请先启动Gazebo仿真，然后再次运行此脚本。"
else
    echo "找到以下摄像头相关话题:"
    echo ""
    echo "$CAMERA_TOPICS" | sort
    echo ""
    echo "如果找到了话题，请在scripts/camera_viewer.py中更新话题名称。"
fi

echo ""
echo "=========================================="

