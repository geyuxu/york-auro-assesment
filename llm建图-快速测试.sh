#!/bin/bash
# LLM 建图 - 快速测试脚本

echo "🤖 LLM 建图 - 快速测试"
echo "================================"
echo ""

# 检查环境
echo "📦 检查依赖..."
python3 -c "import openai" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ openai 库未安装"
    echo "   安装命令: pip install openai"
    exit 1
fi
echo "✅ openai 库已安装"
echo ""

# 检查 API Key
echo "⚙️  检查配置..."

if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ 未设置 OPENAI_API_KEY"
    echo ""
    echo "请设置 API Key："
    echo "  export OPENAI_API_KEY='sk-your-key-here'"
    echo ""
    echo "获取 API Key："
    echo "  - OpenAI: https://platform.openai.com/"
    echo "  - DeepSeek: https://platform.deepseek.com/"
    echo "  - 通义千问: https://dashscope.aliyun.com/"
    echo ""
    exit 1
fi
echo "✅ OPENAI_API_KEY 已设置: ${OPENAI_API_KEY:0:10}..."

if [ -z "$OPENAI_API_BASE" ]; then
    echo "⚠️  未设置 OPENAI_API_BASE，使用默认值"
    export OPENAI_API_BASE="https://api.openai.com/v1"
fi
echo "✅ OPENAI_API_BASE: $OPENAI_API_BASE"

if [ -z "$OPENAI_MODEL" ]; then
    echo "⚠️  未设置 OPENAI_MODEL，使用默认值"
    export OPENAI_MODEL="gpt-4o-mini"
fi
echo "✅ OPENAI_MODEL: $OPENAI_MODEL"

echo ""
echo "🎉 配置检查完成！"
echo ""
echo "📋 启动流程："
echo ""
echo "终端 1 - 启动仿真："
echo "  cd /workspaces/AURO2025 && source install/setup.bash"
echo "  ros2 launch assessment assessment.launch.py \\"
echo "    num_robots:=1 use_nav2:=false headless:=false random_seed:=42"
echo ""
echo "终端 2 - 启动 SLAM："
echo "  cd /workspaces/AURO2025 && source install/setup.bash"
echo "  ros2 launch assessment cartographer.launch.py \\"
echo "    ns:=robot1 use_sim_time:=true use_rviz:=true"
echo ""
echo "终端 3 - 启动 LLM 建图（此终端）："
echo "  ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1"
echo ""
echo "终端 4 - 保存地图（2小时后）："
echo "  cd /workspaces/AURO2025 && source install/setup.bash"
echo "  ros2 run nav2_map_server map_saver_cli -f solution/config/map2 \\"
echo "    --ros-args -r map:=/robot1/map"
echo ""
echo "⏰ 建图时长: 2 小时（7200 秒）"
echo "🚪 优化: 降低阈值到 0.5m，可通过门口"
echo ""
