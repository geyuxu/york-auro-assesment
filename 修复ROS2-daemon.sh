#!/bin/bash
# 修复 ROS2 daemon 问题

echo "🔧 修复 ROS2 daemon"
echo "================================"
echo ""

echo "步骤 1: 停止 ROS2 daemon..."
ros2 daemon stop
sleep 2

echo "步骤 2: 启动 ROS2 daemon..."
ros2 daemon start
sleep 2

echo "步骤 3: 验证 ROS2 daemon..."
if ros2 daemon status &> /dev/null; then
    echo "✅ ROS2 daemon 运行正常"
    echo ""
    echo "现在可以运行："
    echo "  bash 保存Cartographer状态.sh robot1 你的地图名称"
else
    echo "❌ ROS2 daemon 仍有问题"
    echo ""
    echo "请尝试完全重启 ROS2 环境："
    echo "  1. 关闭所有 ROS2 节点（Ctrl+C）"
    echo "  2. 重新 source 环境"
    echo "  3. 重新启动仿真和 Cartographer"
fi

echo ""
