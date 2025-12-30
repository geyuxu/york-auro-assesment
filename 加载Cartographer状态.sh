#!/bin/bash
# 加载 Cartographer SLAM 状态（用于继续建图）

echo "🔄 加载 Cartographer SLAM 状态"
echo "================================"
echo ""

# 配置
ROBOT_NS="${1:-robot1}"
STATE_FILE="${2}"
STATE_DIR="/workspaces/AURO2025/solution/cartographer_states"

# 检查 ROS2 环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ 错误：未找到 ros2 命令"
    echo "   请先执行: source install/setup.bash"
    exit 1
fi

# 如果未提供状态文件，列出可用的状态
if [ -z "$STATE_FILE" ]; then
    echo "📂 可用的 SLAM 状态："
    echo ""

    if [ -d "$STATE_DIR" ] && [ "$(ls -A $STATE_DIR/*.pbstream 2>/dev/null)" ]; then
        # 找到所有 .pbstream 文件
        statefiles=($(ls -t $STATE_DIR/*.pbstream 2>/dev/null | grep -v "latest.pbstream"))

        if [ ${#statefiles[@]} -eq 0 ]; then
            echo "   ⚠️  未找到已保存的状态"
            echo ""
            echo "💡 使用方法："
            echo "   bash 保存Cartographer状态.sh $ROBOT_NS [状态名称]"
            exit 1
        fi

        for i in "${!statefiles[@]}"; do
            statefile="${statefiles[$i]}"
            basename=$(basename "$statefile" .pbstream)
            timestamp=$(stat -c %y "$statefile" | cut -d' ' -f1,2 | cut -d'.' -f1)
            filesize=$(ls -lh "$statefile" | awk '{print $5}')

            echo "   [$((i+1))] $basename"
            echo "       时间: $timestamp"
            echo "       大小: $filesize"
            echo "       路径: $statefile"
            echo ""
        done

        echo -n "请选择状态编号 [1-${#statefiles[@]}] 或按 Enter 使用最新: "
        read choice

        if [ -z "$choice" ]; then
            choice=1
        fi

        if [ "$choice" -ge 1 ] && [ "$choice" -le "${#statefiles[@]}" ]; then
            STATE_FILE="${statefiles[$((choice-1))]}"
            echo ""
            echo "✅ 已选择: $(basename "$STATE_FILE")"
        else
            echo "❌ 无效选择"
            exit 1
        fi
    else
        echo "   ⚠️  目录为空或不存在: $STATE_DIR"
        echo ""
        echo "💡 请先使用保存脚本创建状态："
        echo "   bash 保存Cartographer状态.sh $ROBOT_NS [状态名称]"
        exit 1
    fi
fi

echo ""
echo "📋 加载配置："
echo "  机器人命名空间: /$ROBOT_NS"
echo "  状态文件: $STATE_FILE"
echo ""

# 检查文件是否存在
if [ ! -f "$STATE_FILE" ]; then
    echo "❌ 错误：状态文件不存在: $STATE_FILE"
    exit 1
fi

# 检查仿真是否已启动
echo "🔍 检查仿真环境..."
if ! ros2 topic list 2>/dev/null | grep -q "/$ROBOT_NS/scan"; then
    echo "⚠️  警告：未检测到机器人话题"
    echo ""
    echo "⚠️  请先在终端1启动仿真:"
    echo "   ros2 launch assessment assessment.launch.py \\"
    echo "     num_robots:=1 use_nav2:=false headless:=false random_seed:=42"
    echo ""
    read -p "仿真已启动? 按 Enter 继续，或 Ctrl+C 取消..."
fi

echo ""
echo "🚀 启动 Cartographer (加载状态)..."
echo ""
echo "⚠️  重要提示："
echo "  1. Cartographer 将在此终端运行（不要关闭）"
echo "  2. RViz 会自动打开"
echo "  3. 启动完成后，在新终端运行 LLM 建图节点"
echo ""
echo "💡 下一步："
echo "  在新终端执行:"
echo "    cd /workspaces/AURO2025 && source install/setup.bash"
echo "    export OPENAI_API_KEY=\"你的API密钥\""
echo "    ros2 run solution llm_mapper --ros-args --remap __ns:=/$ROBOT_NS"
echo ""
echo "按 Enter 启动 Cartographer..."
read

# 确保使用正确的工作空间
WORKSPACE_DIR="/workspaces/AURO2025"

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "❌ 错误：找不到工作空间 setup.bash"
    echo "   路径: $WORKSPACE_DIR/install/setup.bash"
    echo ""
    echo "请先构建工作空间："
    echo "   cd $WORKSPACE_DIR"
    echo "   colcon build --symlink-install"
    exit 1
fi

echo "🔧 加载工作空间环境..."
source "$WORKSPACE_DIR/install/setup.bash"

# 启动 Cartographer 并加载状态
exec ros2 launch assessment cartographer.launch.py \
  ns:=$ROBOT_NS \
  use_sim_time:=true \
  use_rviz:=true \
  load_state_filename:="$STATE_FILE"
