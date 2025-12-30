#!/bin/bash
# 保存地图和机器人位置脚本

echo "🗺️  保存地图和机器人位置"
echo "================================"
echo ""

# 配置
ROBOT_NS="${1:-robot1}"  # 机器人命名空间，默认 robot1
SAVE_DIR="/workspaces/AURO2025/solution/saved_maps"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="${2:-map_${TIMESTAMP}}"

# 创建保存目录
mkdir -p "$SAVE_DIR"

echo "📋 配置信息："
echo "  机器人命名空间: /$ROBOT_NS"
echo "  保存目录: $SAVE_DIR"
echo "  地图名称: $MAP_NAME"
echo ""

# 检查 ROS2 环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ 错误：未找到 ros2 命令"
    echo "   请先执行: source install/setup.bash"
    exit 1
fi

# 1. 保存地图
echo "💾 步骤 1/3: 保存地图..."
ros2 run nav2_map_server map_saver_cli \
    -f "$SAVE_DIR/$MAP_NAME" \
    --ros-args -r map:=/$ROBOT_NS/map

if [ $? -eq 0 ]; then
    echo "✅ 地图保存成功: $SAVE_DIR/$MAP_NAME.pgm/yaml"
else
    echo "❌ 地图保存失败"
    exit 1
fi
echo ""

# 2. 获取机器人当前位置
echo "📍 步骤 2/3: 获取机器人位置..."

# 使用 ros2 topic echo 获取 odom 数据
ODOM_DATA=$(timeout 2 ros2 topic echo /$ROBOT_NS/odom geometry_msgs/msg/Odometry --once 2>/dev/null)

if [ -z "$ODOM_DATA" ]; then
    echo "⚠️  警告：无法从 /$ROBOT_NS/odom 获取位置"
    echo "   尝试从 TF 获取..."

    # 尝试从 TF 获取
    TF_DATA=$(timeout 2 ros2 run tf2_ros tf2_echo map $ROBOT_NS/base_footprint 2>/dev/null | head -20)

    if [ -z "$TF_DATA" ]; then
        echo "❌ 无法获取机器人位置"
        exit 1
    fi

    # 从 TF 数据提取位置
    X=$(echo "$TF_DATA" | grep "Translation:" -A 3 | grep "x:" | awk '{print $2}')
    Y=$(echo "$TF_DATA" | grep "Translation:" -A 3 | grep "y:" | awk '{print $2}')
    Z=$(echo "$TF_DATA" | grep "Translation:" -A 3 | grep "z:" | awk '{print $2}')
    QX=$(echo "$TF_DATA" | grep "Rotation:" -A 4 | grep "x:" | awk '{print $2}')
    QY=$(echo "$TF_DATA" | grep "Rotation:" -A 4 | grep "y:" | awk '{print $2}')
    QZ=$(echo "$TF_DATA" | grep "Rotation:" -A 4 | grep "z:" | awk '{print $2}')
    QW=$(echo "$TF_DATA" | grep "Rotation:" -A 4 | grep "w:" | awk '{print $2}')
else
    # 从 odom 数据提取位置
    X=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
    Y=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "y:" | awk '{print $2}')
    Z=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
    QX=$(echo "$ODOM_DATA" | grep -A 4 "orientation:" | grep "x:" | awk '{print $2}')
    QY=$(echo "$ODOM_DATA" | grep -A 4 "orientation:" | grep "y:" | awk '{print $2}')
    QZ=$(echo "$ODOM_DATA" | grep -A 4 "orientation:" | grep "z:" | awk '{print $2}')
    QW=$(echo "$ODOM_DATA" | grep -A 4 "orientation:" | grep "w:" | awk '{print $2}')
fi

# 验证数据
if [ -z "$X" ] || [ -z "$Y" ]; then
    echo "❌ 提取位置数据失败"
    exit 1
fi

echo "✅ 机器人位置："
echo "   位置: x=$X, y=$Y, z=$Z"
echo "   姿态: qx=$QX, qy=$QY, qz=$QZ, qw=$QW"
echo ""

# 3. 保存位置信息到 YAML 文件
echo "💾 步骤 3/3: 保存位置信息..."

POSE_FILE="$SAVE_DIR/${MAP_NAME}_pose.yaml"

cat > "$POSE_FILE" << EOF
# 机器人位置信息
# 保存时间: $(date)
# 机器人: $ROBOT_NS
# 地图: $MAP_NAME

robot_namespace: "$ROBOT_NS"
map_name: "$MAP_NAME"

# 初始位置
initial_pose:
  position:
    x: $X
    y: $Y
    z: $Z
  orientation:
    x: $QX
    y: $QY
    z: $QZ
    w: $QW

# RViz 格式（可直接复制到 RViz 的 2D Pose Estimate）
rviz_pose:
  frame_id: "map"
  pose:
    position: {x: $X, y: $Y, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
EOF

echo "✅ 位置信息保存成功: $POSE_FILE"
echo ""

# 4. 创建快速加载脚本
LOAD_SCRIPT="$SAVE_DIR/load_${MAP_NAME}.sh"

cat > "$LOAD_SCRIPT" << 'EOFSCRIPT'
#!/bin/bash
# 自动生成的加载脚本

MAP_DIR="$(dirname "$0")"
MAP_NAME="MAP_NAME_PLACEHOLDER"
ROBOT_NS="ROBOT_NS_PLACEHOLDER"

echo "🔄 加载地图和位置: $MAP_NAME"
echo ""

# 调用主加载脚本
bash "$(dirname "$MAP_DIR")/加载地图和位置.sh" "$ROBOT_NS" "$MAP_DIR/$MAP_NAME"
EOFSCRIPT

# 替换占位符
sed -i "s|MAP_NAME_PLACEHOLDER|$MAP_NAME|g" "$LOAD_SCRIPT"
sed -i "s|ROBOT_NS_PLACEHOLDER|$ROBOT_NS|g" "$LOAD_SCRIPT"

chmod +x "$LOAD_SCRIPT"

echo "✅ 创建快速加载脚本: $LOAD_SCRIPT"
echo ""

# 5. 保存摘要信息
SUMMARY_FILE="$SAVE_DIR/${MAP_NAME}_summary.txt"

cat > "$SUMMARY_FILE" << EOF
地图保存摘要
=====================================

保存时间: $(date)
机器人: $ROBOT_NS
地图名称: $MAP_NAME

文件列表:
---------
1. 地图图像: ${MAP_NAME}.pgm
2. 地图配置: ${MAP_NAME}.yaml
3. 位置信息: ${MAP_NAME}_pose.yaml
4. 加载脚本: load_${MAP_NAME}.sh
5. 摘要信息: ${MAP_NAME}_summary.txt

机器人位置:
-----------
Position: x=$X, y=$Y, z=$Z
Orientation: qx=$QX, qy=$QY, qz=$QZ, qw=$QW

如何加载:
---------
方法 1 (快捷方式):
  bash $SAVE_DIR/load_${MAP_NAME}.sh

方法 2 (手动):
  bash 加载地图和位置.sh $ROBOT_NS $SAVE_DIR/$MAP_NAME

方法 3 (仅地图):
  ros2 run nav2_map_server map_server \\
    --ros-args -p yaml_filename:=$SAVE_DIR/${MAP_NAME}.yaml

方法 4 (仅位置):
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \\
    "{header: {frame_id: 'map'}, \\
     pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, \\
                   orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}"
EOF

echo "✅ 摘要信息保存成功: $SUMMARY_FILE"
echo ""

# 6. 显示保存结果
echo "🎉 完成！地图和位置已保存"
echo "================================"
echo ""
echo "📂 保存的文件："
ls -lh "$SAVE_DIR" | grep "$MAP_NAME"
echo ""
echo "📖 查看摘要信息："
echo "  cat $SUMMARY_FILE"
echo ""
echo "🚀 快速加载："
echo "  bash $LOAD_SCRIPT"
echo ""
