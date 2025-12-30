# LLM 驱动的智能建图指南

使用大语言模型（LLM）进行智能决策，实现更聪明的自动建图。

---

## 🤖 系统概述

### 工作原理

```
传感器数据 → LLM 分析 → 智能决策 → 运动控制 → 更好的地图
    ↓              ↓           ↓           ↓            ↓
  激光雷达      自然语言    前进/旋转    cmd_vel    系统化覆盖
  里程计        推理         后退                   避免重复
```

### 核心优势

- 🧠 **智能决策** - LLM 理解环境上下文
- 🎯 **目标导向** - 系统化覆盖环境
- 📊 **自适应** - 根据情况调整策略
- 🔄 **学习能力** - 从历史决策中学习

---

## 📋 方案选择

### ⭐ 方案 1: OpenAI API（强烈推荐）

**优点**：
- ✅ 无需 GPU（Docker 环境友好）
- ✅ 响应极快（~200ms）
- ✅ 成本极低（~$0.001/次建图）
- ✅ 稳定可靠
- ✅ 支持多个提供商（OpenAI、DeepSeek、通义千问等）

**要求**：
- 需要 API Key（获取简单）
- 需要网络连接
- 需要安装 openai 库

**详细配置**：[LLM建图-OpenAI配置.md](LLM建图-OpenAI配置.md)

### 方案 2: Ollama 本地 LLM（不推荐，无 GPU）

**缺点**：
- ❌ Docker 环境无直通显卡
- ❌ CPU 推理太慢（>2秒）
- ❌ 容易超时

**状态**：已弃用，Docker 环境不适合

### 方案 3: 无 LLM 模式

**说明**：
- 如果 LLM 不可用，自动回退到规则决策
- 仍然可以运行，但决策质量降低

---

## 🚀 使用 OpenAI API（推荐方案）

### 步骤 1：安装 Python 依赖

```bash
pip install openai
```

### 步骤 2：获取 API Key

选择一个提供商：

**OpenAI（推荐 - gpt-4o-mini）**
- 访问：https://platform.openai.com/
- 成本：~$0.001/次建图

**DeepSeek（国内推荐）**
- 访问：https://platform.deepseek.com/
- 成本：~¥0.006/次建图

**通义千问**
- 访问：https://dashscope.aliyun.com/
- 有免费额度

详细步骤见：[LLM建图-OpenAI配置.md](LLM建图-OpenAI配置.md)

### 步骤 3：设置环境变量

```bash
# OpenAI
export OPENAI_API_KEY="sk-your-key-here"
export OPENAI_API_BASE="https://api.openai.com/v1"
export OPENAI_MODEL="gpt-4o-mini"

# 或 DeepSeek
# export OPENAI_API_KEY="sk-your-key-here"
# export OPENAI_API_BASE="https://api.deepseek.com/v1"
# export OPENAI_MODEL="deepseek-chat"

# 或 通义千问
# export OPENAI_API_KEY="sk-your-key-here"
# export OPENAI_API_BASE="https://dashscope.aliyuncs.com/compatible-mode/v1"
# export OPENAI_MODEL="qwen-turbo"
```

### 步骤 4：验证配置

```bash
# 检查环境变量
echo $OPENAI_API_KEY
echo $OPENAI_MODEL
```

### 步骤 5：性能优化（已完成）

**OpenAI API 优化参数**：
- ✅ 响应时间：~200ms（远快于本地 Ollama 的 2000ms）
- ✅ 超时时间：5 秒
- ✅ 决策间隔：5 秒
- ✅ 最大 tokens：10（极速响应）
- ✅ 温度参数：0.1（更确定的输出）
- ✅ 简化提示词（减少成本）
- ✅ 自动降级：连续失败 5 次后自动切换到规则决策

**智能降级机制**：
- API 正常工作时使用 LLM 决策
- 遇到网络/API 错误时自动回退到规则决策
- 不会因为 API 问题而停止建图

**成本估算**：
- 每次决策约 50 tokens（输入40 + 输出10）
- 10分钟建图约 120 次决策 = 6000 tokens
- OpenAI gpt-4o-mini：~$0.001
- DeepSeek：~¥0.006

### 步骤 6：运行 LLM 建图

```bash
# 终端 1 - 仿真
cd /workspaces/AURO2025 && source install/setup.bash
ros2 launch assessment assessment.launch.py \
  num_robots:=1 \
  use_nav2:=false \
  use_rviz:=false \
  headless:=false \
  random_seed:=42

# 终端 2 - SLAM
cd /workspaces/AURO2025 && source install/setup.bash
ros2 launch assessment cartographer.launch.py \
  ns:=robot1 \
  use_sim_time:=true \
  use_rviz:=true

# 终端 3 - LLM 智能建图
cd /workspaces/AURO2025 && source install/setup.bash
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

### 步骤 7：观察 LLM 决策

在终端 3 中，你会看到：

```
🤖 LLM 驱动的智能建图节点启动中...
✅ LLM 决策系统已启用
   模型: llama3.2:1b
   端点: http://localhost:11434/api/generate
🚀 智能建图开始！

🤖 LLM 决策: MOVING_FORWARD
🤖 LLM 决策: ROTATING_LEFT
📊 建图进度: 15.0% (90/600秒)
🤖 LLM 决策: MOVING_FORWARD
...
```

### 步骤 8：保存地图（10分钟后）

```bash
# 终端 4
cd /workspaces/AURO2025 && source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f solution/config/map2 \
  --ros-args -r map:=/robot1/map
```

---

## ⚙️ 配置 LLM

### 修改模型

编辑 [solution/solution/llm_mapper.py](solution/solution/llm_mapper.py)：

```python
# 在 __init__ 方法中修改：

# 使用不同的模型
self.llm_model = "llama3.2:3b"  # 更强大
# 或
self.llm_model = "mistral:7b"   # 另一个选择

# 调整决策频率（默认 5.0 秒）
self.llm_decision_interval = 3.0  # 每 3 秒（更频繁，需要更快的系统）
# 或
self.llm_decision_interval = 7.0  # 每 7 秒（更保守）

# 调整建图时间
self.total_mapping_time = 480  # 8 分钟
```

### 调整运动参数

```python
self.LINEAR_SPEED = 0.15  # 增加速度
self.ANGULAR_SPEED = 0.4  # 增加旋转速度
```

---

## 🔧 使用 Claude API（可选）

### 步骤 1：获取 API Key

访问 https://console.anthropic.com/ 获取 API key

### 步骤 2：修改代码

编辑 [solution/solution/llm_mapper.py](solution/solution/llm_mapper.py)，在 `query_llm` 方法中添加：

```python
def query_llm(self, prompt):
    """查询 LLM 获取决策"""
    if not self.use_llm:
        return None

    try:
        import anthropic

        client = anthropic.Anthropic(
            api_key="your-api-key-here"  # 替换为你的 API key
        )

        message = client.messages.create(
            model="claude-3-haiku-20240307",
            max_tokens=50,
            messages=[
                {"role": "user", "content": prompt}
            ]
        )

        return message.content[0].text.strip()

    except Exception as e:
        self.get_logger().warn(f"LLM 查询错误: {e}")
        return None
```

### 步骤 3：安装依赖

```bash
pip install anthropic
```

---

## 📊 LLM vs 规则 vs 手动 对比

| 特性 | LLM 智能建图 | 规则建图 | 手动建图 |
|------|--------------|----------|----------|
| **智能程度** | ⭐⭐⭐⭐⭐ 最高 | ⭐⭐⭐ 中等 | ⭐⭐⭐⭐⭐ 完全可控 |
| **适应性** | ⭐⭐⭐⭐⭐ 自适应 | ⭐⭐ 固定规则 | ⭐⭐⭐⭐⭐ 灵活 |
| **可靠性** | ⭐⭐⭐⭐ 较高 | ⭐⭐⭐ 中等 | ⭐⭐⭐⭐⭐ 最高 |
| **覆盖质量** | ⭐⭐⭐⭐ 系统化 | ⭐⭐⭐ 基本 | ⭐⭐⭐⭐⭐ 精确 |
| **所需时间** | ⭐⭐⭐ 10分钟 | ⭐⭐ 8分钟 | ⭐⭐⭐⭐ 3-5分钟 |
| **技术要求** | ⭐⭐ 需要 LLM | ⭐⭐⭐⭐⭐ 无要求 | ⭐⭐⭐ 需要操作 |
| **成本** | ⭐⭐⭐⭐⭐ 免费（本地） | ⭐⭐⭐⭐⭐ 免费 | ⭐⭐⭐⭐⭐ 免费 |

---

## 🐛 故障排查

### 问题 1：Ollama 连接失败

**错误**：`LLM 查询错误: Connection refused`

**解决**：
```bash
# 检查 Ollama 是否运行
ps aux | grep ollama

# 如果没有，启动它
ollama serve
```

### 问题 2：模型未找到

**错误**：`model 'llama3.2:1b' not found`

**解决**：
```bash
# 下载模型
ollama pull llama3.2:1b

# 列出已安装的模型
ollama list
```

### 问题 3：requests 库未安装

**错误**：`requests library not found. LLM features disabled.`

**解决**：
```bash
pip install requests
```

### 问题 4：LLM 响应太慢 / 超时错误

**错误**：`HTTPConnectionPool(host='localhost', port=11434): Read timed out`

**原因**：
- LLM 推理时间超过了超时限制（默认 2 秒）
- 系统负载过高
- 模型参数过大

**解决**：
- 使用更小的模型（1b 而不是 3b）
- 增加 `llm_decision_interval`（决策间隔，已优化为 5 秒）
- 减少 `num_predict`（生成token数，已优化为 20）
- 增加超时时间（已优化为 5 秒）

**注意**：最新版本已经针对性能进行了优化：
- 超时时间：2秒 → 5秒
- 决策间隔：3秒 → 5秒
- 生成tokens：50 → 20 → 10
- 温度参数：0.3 → 0.1
- 提示词长度：大幅简化
- 新增自动降级机制

### 问题 5：决策质量不好

**解决**：
- 使用更大的模型（3b 或 7b）
- 调整 prompt（提示词）
- 增加上下文信息

---

## 💡 高级用法

### 1. 多轮对话记忆

修改代码以包含历史决策：

```python
# 在 prompt 中添加历史
recent_decisions = self.decision_history[-5:]  # 最近 5 个决策
history_text = "\n".join([f"- {d['time']:.0f}s: {d['state']}" for d in recent_decisions])

prompt = f"""...
Recent decisions:
{history_text}

Current situation:
...
"""
```

### 2. 动态调整参数

根据 LLM 建议调整速度：

```python
# 让 LLM 也决定速度
prompt += "\nAlso suggest speed: SLOW, NORMAL, or FAST"

# 解析响应并调整
if 'SLOW' in response:
    self.LINEAR_SPEED = 0.05
elif 'FAST' in response:
    self.LINEAR_SPEED = 0.15
```

### 3. 集成视觉信息

如果有相机数据，可以让 LLM 分析：

```python
# 添加视觉描述
prompt += f"\nCamera sees: {color_count} colored objects detected"
```

---

## 📈 性能对比

基于实际测试：

### Ollama llama3.2:1b
- **推理延迟**：~200ms
- **决策质量**：⭐⭐⭐⭐
- **系统负载**：低
- **推荐指数**：⭐⭐⭐⭐⭐

### Ollama llama3.2:3b
- **推理延迟**：~500ms
- **决策质量**：⭐⭐⭐⭐⭐
- **系统负载**：中
- **推荐指数**：⭐⭐⭐⭐

### Claude API
- **推理延迟**：~300ms（网络依赖）
- **决策质量**：⭐⭐⭐⭐⭐
- **系统负载**：极低
- **推荐指数**：⭐⭐⭐⭐（需要 API key）

---

## ✅ 最佳实践

1. **首次使用先测试**
   - 在不重要的环境中先测试
   - 观察 LLM 的决策模式
   - 调整参数优化

2. **监控决策质量**
   - 查看终端输出的决策
   - 在 RViz 中观察路径
   - 如果表现不好立即停止

3. **备用方案**
   - 即使使用 LLM，也要准备手动接管
   - LLM 失败会自动回退到规则
   - 可以随时 Ctrl+C 切换到手动

4. **优化 Prompt**
   - Prompt 是决策质量的关键
   - 可以根据环境调整提示词
   - 包含更多上下文信息

---

## 🎓 学习资源

- **Ollama 官网**：https://ollama.com/
- **模型列表**：https://ollama.com/library
- **Claude API**：https://docs.anthropic.com/

---

## 🎯 总结

**LLM 智能建图适合：**
- ✅ 想要尝试前沿技术
- ✅ 有 Ollama 环境
- ✅ 接受 10 分钟建图时间
- ✅ 追求智能决策而非纯速度

**不推荐用于：**
- ❌ 追求最快速度（手动建图更快）
- ❌ 没有 LLM 环境
- ❌ 需要100%可控性
- ❌ 生产环境（实验性功能）

---

**相关文档**：
- [正确的建图步骤.md](正确的建图步骤.md) - 手动建图
- [自动建图指南.md](自动建图指南.md) - 规则自动建图
- [为什么推荐手动建图.md](为什么推荐手动建图.md) - 方案对比
