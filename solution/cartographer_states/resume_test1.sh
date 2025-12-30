#!/bin/bash
# 自动生成的 Cartographer 状态恢复脚本

STATE_FILE="/workspaces/AURO2025/solution/cartographer_states/test1.pbstream"
ROBOT_NS="robot1"

echo "🔄 恢复 Cartographer SLAM 状态"
echo "================================"
echo ""
echo "状态文件: $STATE_FILE"
echo "机器人: $ROBOT_NS"
echo ""
echo "⚠️  请确保已在终端1启动仿真:"
echo "   ros2 launch assessment assessment.launch.py \\"
echo "     num_robots:=1 use_nav2:=false headless:=false random_seed:=42"
echo ""
read -p "仿真已启动? 按 Enter 继续..."

echo ""
echo "🚀 启动 Cartographer (加载状态)..."

# 启动 Cartographer 并加载状态
ros2 launch assessment cartographer.launch.py \
  ns:=$ROBOT_NS \
  use_sim_time:=true \
  use_rviz:=true \
  load_state_filename:="$STATE_FILE"

