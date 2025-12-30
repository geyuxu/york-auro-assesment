#!/bin/bash
# 加载地图和机器人位置脚本

echo "🔄 加载地图和机器人位置"
echo "================================"
echo ""

# 配置
ROBOT_NS="${1:-robot1}"  # 机器人命名空间，默认 robot1
MAP_FILE="${2}"           # 地图文件路径（不含扩展名）
SAVE_DIR="/workspaces/AURO2025/solution/saved_maps"

# 检查 ROS2 环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ 错误：未找到 ros2 命令"
    echo "   请先执行: source install/setup.bash"
    exit 1
fi

# 如果未提供地图文件，列出可用的地图
if [ -z "$MAP_FILE" ]; then
    echo "📂 可用的地图："
    echo ""

    if [ -d "$SAVE_DIR" ] && [ "$(ls -A $SAVE_DIR/*.yaml 2>/dev/null)" ]; then
        # 找到所有地图文件（排除 pose 文件）
        mapfiles=($(ls -t $SAVE_DIR/*.yaml 2>/dev/null | grep -v "_pose.yaml"))

        if [ ${#mapfiles[@]} -eq 0 ]; then
            echo "   ⚠️  未找到已保存的地图"
            echo ""
            echo "💡 使用方法："
            echo "   bash 保存地图和位置.sh $ROBOT_NS [地图名称]"
            exit 1
        fi

        for i in "${!mapfiles[@]}"; do
            mapfile="${mapfiles[$i]}"
            basename=$(basename "$mapfile" .yaml)
            timestamp=$(stat -c %y "$mapfile" | cut -d' ' -f1,2 | cut -d'.' -f1)

            # 检查是否有对应的 pose 文件
            posefile="${SAVE_DIR}/${basename}_pose.yaml"
            if [ -f "$posefile" ]; then
                echo "   [$((i+1))] $basename"
                echo "       时间: $timestamp"
                echo "       文件: $mapfile"
            fi
        done

        echo ""
        echo -n "请选择地图编号 [1-${#mapfiles[@]}] 或按 Enter 使用最新: "
        read choice

        if [ -z "$choice" ]; then
            choice=1
        fi

        if [ "$choice" -ge 1 ] && [ "$choice" -le "${#mapfiles[@]}" ]; then
            selected_map="${mapfiles[$((choice-1))]}"
            MAP_FILE="${selected_map%.yaml}"
            echo ""
            echo "✅ 已选择: $(basename "$MAP_FILE")"
        else
            echo "❌ 无效选择"
            exit 1
        fi
    else
        echo "   ⚠️  目录为空或不存在: $SAVE_DIR"
        echo ""
        echo "💡 请先使用保存脚本创建地图："
        echo "   bash 保存地图和位置.sh $ROBOT_NS [地图名称]"
        exit 1
    fi
fi

echo ""
echo "📋 加载配置："
echo "  机器人命名空间: /$ROBOT_NS"
echo "  地图文件: $MAP_FILE.yaml"
echo ""

# 检查文件是否存在
if [ ! -f "${MAP_FILE}.yaml" ]; then
    echo "❌ 错误：地图文件不存在: ${MAP_FILE}.yaml"
    exit 1
fi

if [ ! -f "${MAP_FILE}.pgm" ]; then
    echo "❌ 错误：地图图像不存在: ${MAP_FILE}.pgm"
    exit 1
fi

POSE_FILE="${MAP_FILE}_pose.yaml"
if [ ! -f "$POSE_FILE" ]; then
    echo "⚠️  警告：未找到位置文件: $POSE_FILE"
    echo "   将只加载地图，不设置机器人位置"
    POSE_FILE=""
fi

# 1. 启动地图服务器
echo "🗺️  步骤 1/2: 启动地图服务器..."

# 检查是否已有地图服务器运行
if ros2 node list 2>/dev/null | grep -q "/$ROBOT_NS/map_server"; then
    echo "⚠️  检测到已运行的地图服务器，正在停止..."
    # 尝试优雅关闭（如果可能的话）
    pkill -f "map_server.*$ROBOT_NS" 2>/dev/null
    sleep 2
fi

# 启动新的地图服务器（后台运行）
ros2 run nav2_map_server map_server \
    --ros-args \
    -r __ns:=/$ROBOT_NS \
    -p yaml_filename:="${MAP_FILE}.yaml" \
    -p use_sim_time:=true &

MAP_SERVER_PID=$!

# 等待地图服务器启动
sleep 3

# 检查地图服务器是否成功启动
if ! ps -p $MAP_SERVER_PID > /dev/null; then
    echo "❌ 地图服务器启动失败"
    exit 1
fi

# 检查 /map 话题是否发布
if timeout 5 ros2 topic echo /$ROBOT_NS/map nav_msgs/msg/OccupancyGrid --once > /dev/null 2>&1; then
    echo "✅ 地图服务器已启动 (PID: $MAP_SERVER_PID)"
else
    echo "❌ 地图服务器未正常发布地图话题"
    kill $MAP_SERVER_PID 2>/dev/null
    exit 1
fi

echo ""

# 2. 加载机器人位置
if [ -n "$POSE_FILE" ]; then
    echo "📍 步骤 2/2: 设置机器人位置..."

    # 从 YAML 文件提取位置信息
    X=$(grep "x:" "$POSE_FILE" | head -1 | awk '{print $2}')
    Y=$(grep "y:" "$POSE_FILE" | sed -n '2p' | awk '{print $2}')
    Z=$(grep "z:" "$POSE_FILE" | sed -n '3p' | awk '{print $2}')
    QX=$(grep "x:" "$POSE_FILE" | sed -n '2p' | awk '{print $2}')
    QY=$(grep "y:" "$POSE_FILE" | sed -n '4p' | awk '{print $2}')
    QZ=$(grep "z:" "$POSE_FILE" | sed -n '4p' | awk '{print $2}')
    QW=$(grep "w:" "$POSE_FILE" | tail -1 | awk '{print $2}')

    # 验证数据
    if [ -z "$X" ] || [ -z "$Y" ] || [ -z "$QW" ]; then
        echo "❌ 无法从 $POSE_FILE 读取位置数据"
        echo "   地图已加载，但未设置机器人位置"
    else
        echo "✅ 读取位置："
        echo "   位置: x=$X, y=$Y, z=$Z"
        echo "   姿态: qx=$QX, qy=$QY, qz=$QZ, qw=$QW"
        echo ""

        # 发布初始位置到 /initialpose 话题（用于 AMCL）
        echo "📡 发布初始位置到 /$ROBOT_NS/initialpose..."

        ros2 topic pub -1 /$ROBOT_NS/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{
            header: {
                stamp: {sec: 0, nanosec: 0},
                frame_id: 'map'
            },
            pose: {
                pose: {
                    position: {x: $X, y: $Y, z: 0.0},
                    orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
                },
                covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
            }
        }" > /dev/null 2>&1

        if [ $? -eq 0 ]; then
            echo "✅ 初始位置已发布"
        else
            echo "⚠️  发布初始位置失败（话题可能不存在）"
            echo "   如果使用 AMCL，请确保 AMCL 节点正在运行"
        fi
    fi
else
    echo "⏭️  步骤 2/2: 跳过（无位置文件）"
fi

echo ""
echo "🎉 完成！"
echo "================================"
echo ""
echo "📊 状态信息："
echo "  地图服务器 PID: $MAP_SERVER_PID"
echo "  地图话题: /$ROBOT_NS/map"
if [ -n "$POSE_FILE" ]; then
    echo "  初始位置: 已发布到 /$ROBOT_NS/initialpose"
fi
echo ""
echo "🔍 验证命令："
echo "  # 查看地图"
echo "  ros2 run rviz2 rviz2"
echo ""
echo "  # 检查地图话题"
echo "  ros2 topic echo /$ROBOT_NS/map nav_msgs/msg/OccupancyGrid --once"
echo ""
echo "  # 查看 TF 树"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo "🛑 停止地图服务器："
echo "  kill $MAP_SERVER_PID"
echo ""
