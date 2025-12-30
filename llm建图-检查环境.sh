#!/bin/bash
# LLM 建图环境检查脚本

echo "╔════════════════════════════════════════════════════════════════════════════╗"
echo "║                     LLM 智能建图 - 环境检查                                  ║"
echo "╚════════════════════════════════════════════════════════════════════════════╝"
echo ""

# 检查 Ollama
echo "检查 1: Ollama 安装"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if command -v ollama &> /dev/null; then
    echo "✅ Ollama 已安装"
    ollama --version
else
    echo "❌ Ollama 未安装"
    echo ""
    echo "安装方法："
    echo "  curl -fsSL https://ollama.com/install.sh | sh"
    echo ""
fi
echo ""

# 检查 Ollama 服务
echo "检查 2: Ollama 服务状态"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if pgrep -x "ollama" > /dev/null; then
    echo "✅ Ollama 服务正在运行"
else
    echo "⚠️  Ollama 服务未运行"
    echo ""
    echo "启动方法："
    echo "  ollama serve &"
    echo ""
fi
echo ""

# 检查模型
echo "检查 3: 已安装的模型"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if command -v ollama &> /dev/null && pgrep -x "ollama" > /dev/null; then
    ollama list 2>/dev/null || echo "无法获取模型列表"
    echo ""

    # 检查推荐模型
    if ollama list 2>/dev/null | grep -q "llama3.2:1b"; then
        echo "✅ 推荐模型 llama3.2:1b 已安装"
    else
        echo "⚠️  推荐模型 llama3.2:1b 未安装"
        echo ""
        echo "安装方法："
        echo "  ollama pull llama3.2:1b"
        echo ""
    fi
else
    echo "⚠️  无法检查（Ollama 未运行）"
fi
echo ""

# 检查 Python 依赖
echo "检查 4: Python 依赖"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if python3 -c "import requests" 2>/dev/null; then
    echo "✅ requests 库已安装"
else
    echo "❌ requests 库未安装"
    echo ""
    echo "安装方法："
    echo "  pip install requests"
    echo ""
fi
echo ""

# 检查 ROS 包
echo "检查 5: ROS 解决方案包"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cd /workspaces/AURO2025 > /dev/null 2>&1
source install/setup.bash > /dev/null 2>&1
if ros2 pkg executables solution 2>/dev/null | grep -q "llm_mapper"; then
    echo "✅ llm_mapper 节点已编译"
else
    echo "❌ llm_mapper 节点未找到"
    echo ""
    echo "编译方法："
    echo "  colcon build --packages-select solution --symlink-install"
    echo ""
fi
echo ""

# 总结
echo "════════════════════════════════════════════════════════════════════════════"
echo "总结"
echo "════════════════════════════════════════════════════════════════════════════"
echo ""

# 计算状态
ready=true
if ! command -v ollama &> /dev/null; then
    ready=false
fi
if ! pgrep -x "ollama" > /dev/null; then
    ready=false
fi
if ! python3 -c "import requests" 2>/dev/null; then
    ready=false
fi

if [ "$ready" = true ]; then
    echo "✅ 所有检查通过！可以使用 LLM 智能建图"
    echo ""
    echo "启动命令："
    echo "  cd /workspaces/AURO2025"
    echo "  source install/setup.bash"
    echo "  ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1"
    echo ""
else
    echo "⚠️  部分检查失败，请先完成上述安装步骤"
    echo ""
    echo "快速修复："
    echo ""
    echo "  # 1. 安装 Ollama"
    echo "  curl -fsSL https://ollama.com/install.sh | sh"
    echo ""
    echo "  # 2. 启动 Ollama"
    echo "  ollama serve &"
    echo ""
    echo "  # 3. 下载模型"
    echo "  ollama pull llama3.2:1b"
    echo ""
    echo "  # 4. 安装 Python 依赖"
    echo "  pip install requests"
    echo ""
    echo "  # 5. 编译 ROS 包"
    echo "  cd /workspaces/AURO2025"
    echo "  colcon build --packages-select solution --symlink-install"
    echo ""
fi

echo "════════════════════════════════════════════════════════════════════════════"
echo "查看详细文档：cat LLM智能建图指南.md"
echo "════════════════════════════════════════════════════════════════════════════"
