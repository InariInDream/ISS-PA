#!/usr/bin/env zsh
# 测试2D Nav Goal话题是否正常

source ~/catkin_ws/devel/setup.zsh

echo "=========================================="
echo "测试2D Nav Goal话题"
echo "=========================================="
echo ""
echo "1. 检查话题是否存在..."
if rostopic list | grep -q "move_base_simple/goal"; then
    echo "✓ 找到话题: /move_base_simple/goal"
else
    echo "✗ 未找到话题: /move_base_simple/goal"
    echo "  请在Rviz中点击一次2D Nav Goal工具，然后再运行此脚本"
fi

echo ""
echo "2. 监听话题消息（按Ctrl+C停止）..."
echo "   现在请在Rviz中点击2D Nav Goal工具，在地图上点击一个点"
echo ""
rostopic echo /move_base_simple/goal

