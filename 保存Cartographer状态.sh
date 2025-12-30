#!/bin/bash
# 保存 Cartographer SLAM 状态（用于增量建图）

echo "💾 保存 Cartographer SLAM 状态"
echo "================================"
echo ""

# 配置
ROBOT_NS="${1:-robot1}"
SAVE_DIR="/workspaces/AURO2025/solution/cartographer_states"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
STATE_NAME="${2:-state_${TIMESTAMP}}"

# 创建保存目录
mkdir -p "$SAVE_DIR"

echo "📋 配置信息："
echo "  机器人命名空间: /$ROBOT_NS"
echo "  保存目录: $SAVE_DIR"
echo "  状态名称: $STATE_NAME"
echo ""

# 检查 ROS2 环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ 错误：未找到 ros2 命令"
    echo "   请先执行: source install/setup.bash"
    exit 1
fi

# 检查 Cartographer 服务是否可用
echo "🔍 检查 Cartographer write_state 服务..."

# 直接尝试调用服务来检测（更可靠）
# 使用 timeout 避免长时间等待
if timeout 5 ros2 service type /$ROBOT_NS/write_state &> /dev/null; then
    echo "✅ Cartographer 服务已找到"
else
    echo "❌ 错误：Cartographer write_state 服务不可用"
    echo ""
    echo "⚠️  可能的原因："
    echo "   1. Cartographer 未启动"
    echo "   2. 命名空间不匹配（当前使用: /$ROBOT_NS）"
    echo "   3. ROS2 daemon 需要重启"
    echo ""
    echo "💡 解决方法："
    echo ""
    echo "   方法 1 - 重启 ROS2 daemon（推荐）："
    echo "   ros2 daemon stop"
    echo "   ros2 daemon start"
    echo "   # 然后重新运行此脚本"
    echo ""
    echo "   方法 2 - 确保 Cartographer 正在运行："
    echo "   # 在另一个终端执行："
    echo "   ros2 launch assessment cartographer.launch.py \\"
    echo "     ns:=$ROBOT_NS use_sim_time:=true use_rviz:=true"
    echo ""
    echo "   方法 3 - 检查命名空间："
    echo "   # 列出所有可用的 write_state 服务："
    echo "   ros2 service list | grep write_state"
    echo ""
    exit 1
fi
echo ""

# 1. 保存 Cartographer 状态 (.pbstream)
echo "💾 步骤 1/3: 保存 Cartographer 状态..."

PBSTREAM_FILE="$SAVE_DIR/${STATE_NAME}.pbstream"

ros2 service call /$ROBOT_NS/write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$PBSTREAM_FILE', include_unfinished_submaps: true}"

if [ $? -eq 0 ] && [ -f "$PBSTREAM_FILE" ]; then
    echo "✅ Cartographer 状态保存成功: $PBSTREAM_FILE"
    FILE_SIZE=$(ls -lh "$PBSTREAM_FILE" | awk '{print $5}')
    echo "   文件大小: $FILE_SIZE"
else
    echo "❌ Cartographer 状态保存失败"
    exit 1
fi
echo ""

# 2. 保存当前地图（用于可视化）
echo "💾 步骤 2/3: 保存地图图像（用于可视化）..."

MAP_FILE="$SAVE_DIR/${STATE_NAME}_map"

ros2 run nav2_map_server map_saver_cli \
    -f "$MAP_FILE" \
    --ros-args -r map:=/$ROBOT_NS/map 2>/dev/null

if [ -f "${MAP_FILE}.pgm" ]; then
    echo "✅ 地图图像保存成功: ${MAP_FILE}.pgm/.yaml"
else
    echo "⚠️  地图图像保存失败（不影响继续建图）"
fi
echo ""

# 3. 保存元数据
echo "💾 步骤 3/3: 保存元数据..."

METADATA_FILE="$SAVE_DIR/${STATE_NAME}_info.txt"

cat > "$METADATA_FILE" << EOF
Cartographer SLAM 状态保存信息
=====================================

保存时间: $(date)
机器人命名空间: $ROBOT_NS
状态名称: $STATE_NAME

文件列表:
---------
1. SLAM 状态: ${STATE_NAME}.pbstream (主文件)
2. 地图图像: ${STATE_NAME}_map.pgm
3. 地图配置: ${STATE_NAME}_map.yaml
4. 元数据: ${STATE_NAME}_info.txt

文件大小:
---------
$(ls -lh "$SAVE_DIR" | grep "$STATE_NAME")

如何继续建图:
-----------
# 终端 1 - 仿真
ros2 launch assessment assessment.launch.py \\
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42

# 终端 2 - 加载 SLAM 状态并继续建图
bash 加载Cartographer状态.sh $ROBOT_NS $PBSTREAM_FILE

# 终端 3 - LLM 建图
cd /workspaces/AURO2025 && source install/setup.bash
export OPENAI_API_KEY="你的API密钥"
ros2 run solution llm_mapper --ros-args --remap __ns:=/$ROBOT_NS

或者使用快速加载脚本:
bash $SAVE_DIR/resume_${STATE_NAME}.sh
EOF

echo "✅ 元数据保存成功: $METADATA_FILE"
echo ""

# 4. 创建快速恢复脚本
RESUME_SCRIPT="$SAVE_DIR/resume_${STATE_NAME}.sh"

cat > "$RESUME_SCRIPT" << 'EOFSCRIPT'
#!/bin/bash
# 自动生成的 Cartographer 状态恢复脚本

STATE_FILE="STATE_FILE_PLACEHOLDER"
ROBOT_NS="ROBOT_NS_PLACEHOLDER"

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

EOFSCRIPT

# 替换占位符
sed -i "s|STATE_FILE_PLACEHOLDER|$PBSTREAM_FILE|g" "$RESUME_SCRIPT"
sed -i "s|ROBOT_NS_PLACEHOLDER|$ROBOT_NS|g" "$RESUME_SCRIPT"

chmod +x "$RESUME_SCRIPT"

echo "✅ 快速恢复脚本创建成功: $RESUME_SCRIPT"
echo ""

# 5. 创建最新状态的符号链接
LATEST_LINK="$SAVE_DIR/latest.pbstream"
if [ -L "$LATEST_LINK" ]; then
    rm "$LATEST_LINK"
fi
ln -s "$PBSTREAM_FILE" "$LATEST_LINK"

echo "✅ 已更新最新状态链接: $LATEST_LINK"
echo ""

# 6. 显示完成信息
echo "🎉 完成！Cartographer 状态已保存"
echo "================================"
echo ""
echo "📂 保存的文件："
ls -lh "$SAVE_DIR" | grep "$STATE_NAME"
echo ""
echo "📖 查看详细信息："
echo "  cat $METADATA_FILE"
echo ""
echo "🚀 继续建图："
echo "  方法 1 (推荐): bash $RESUME_SCRIPT"
echo "  方法 2 (手动): bash 加载Cartographer状态.sh $ROBOT_NS $PBSTREAM_FILE"
echo ""
echo "💡 提示："
echo "  - .pbstream 文件包含完整的 SLAM 状态"
echo "  - 可以从此状态继续建图"
echo "  - 定期保存以避免进度丢失"
echo ""
