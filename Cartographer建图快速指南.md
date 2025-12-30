# Cartographer 建图快速指南

## 🚀 首次建图（3 个终端）

```bash
# 终端 1 - 仿真
ros2 launch assessment assessment.launch.py \
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42

# 终端 2 - SLAM
ros2 launch assessment cartographer.launch.py \
  ns:=robot1 use_sim_time:=true use_rviz:=true

# 终端 3 - LLM 建图
cd /workspaces/AURO2025 && source install/setup.bash
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

---

## 💾 保存进度（任意时刻）

```bash
# 终端 4
bash 保存Cartographer状态.sh robot1 我的地图名称
```

---

## 🔄 继续建图（从保存的状态）

```bash
# 终端 1 - 仿真（同首次建图）
ros2 launch assessment assessment.launch.py \
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42

# 终端 2 - 加载 SLAM 状态
bash 加载Cartographer状态.sh
# 选择要加载的状态

# 终端 3 - LLM 建图（同首次建图）
cd /workspaces/AURO2025 && source install/setup.bash
export OPENAI_API_KEY="sk-proj-你的密钥"
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

---

## 📁 保存的文件

**位置**：`/workspaces/AURO2025/solution/cartographer_states/`

**主要文件**：
- `状态名称.pbstream` - SLAM 状态（用于继续建图）
- `状态名称_map.pgm/.yaml` - 地图图像（用于可视化）
- `resume_状态名称.sh` - 快速恢复脚本

---

## ⚡ 快捷方式

### 使用自动生成的恢复脚本
```bash
# 终端 1 - 仿真
ros2 launch assessment assessment.launch.py \
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42

# 终端 2 - 快速恢复
bash /workspaces/AURO2025/solution/cartographer_states/resume_我的地图名称.sh
```

---

## 🔍 检查保存的状态

```bash
# 查看所有保存的状态
ls -lht /workspaces/AURO2025/solution/cartographer_states/

# 查看状态详细信息
cat /workspaces/AURO2025/solution/cartographer_states/状态名称_info.txt
```

---

## ⚠️  重要提示

1. **保存前必须**：Cartographer 正在运行
2. **加载前必须**：先启动仿真环境
3. **继续建图**：加载状态后在新终端启动 LLM 建图
4. **定期保存**：建议每 30-60 分钟保存一次

---

## 📖 完整文档

详细说明请参考：[Cartographer增量建图指南.md](./Cartographer增量建图指南.md)
