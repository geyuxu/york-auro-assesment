# Cartographer 增量建图指南

## 🎯 功能说明

使用 Cartographer 的 `.pbstream` 状态文件实现增量建图，可以：
- 💾 保存当前 SLAM 状态
- 🔄 从保存的状态继续建图
- ⏸️ 随时暂停和恢复建图进度

---

## 🚀 完整工作流程

### 场景 1：首次建图

#### 步骤 1：启动仿真环境
```bash
# 终端 1
ros2 launch assessment assessment.launch.py \
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42
```

#### 步骤 2：启动 Cartographer SLAM
```bash
# 终端 2
ros2 launch assessment cartographer.launch.py \
  ns:=robot1 use_sim_time:=true use_rviz:=true
```

#### 步骤 3：启动 LLM 建图
```bash
# 终端 3
cd /workspaces/AURO2025 && source install/setup.bash
export OPENAI_API_KEY="sk-proj-你的密钥"
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

#### 步骤 4：建图一段时间后保存状态
```bash
# 终端 4（建图进行中随时可以保存）
cd /workspaces/AURO2025
source install/setup.bash
bash 保存Cartographer状态.sh robot1 我的地图_第一阶段
```

**输出示例：**
```
💾 保存 Cartographer SLAM 状态
================================

📋 配置信息：
  机器人命名空间: /robot1
  保存目录: /workspaces/AURO2025/solution/cartographer_states
  状态名称: 我的地图_第一阶段

✅ Cartographer 状态保存成功: /workspaces/AURO2025/solution/cartographer_states/我的地图_第一阶段.pbstream
   文件大小: 2.3M

✅ 地图图像保存成功: 我的地图_第一阶段_map.pgm/.yaml

🎉 完成！Cartographer 状态已保存
```

---

### 场景 2：从保存的状态继续建图

#### 步骤 1：启动仿真环境（同场景1）
```bash
# 终端 1
ros2 launch assessment assessment.launch.py \
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42
```

#### 步骤 2：加载 SLAM 状态并启动 Cartographer
```bash
# 终端 2
cd /workspaces/AURO2025
source install/setup.bash
bash 加载Cartographer状态.sh
# 选择要加载的状态（或直接指定文件路径）
```

**交互式选择界面：**
```
🔄 加载 Cartographer SLAM 状态
================================

📂 可用的 SLAM 状态：

   [1] 我的地图_第一阶段
       时间: 2025-12-30 14:30:00
       大小: 2.3M
       路径: /workspaces/AURO2025/solution/cartographer_states/我的地图_第一阶段.pbstream

   [2] state_20251229_163000
       时间: 2025-12-29 16:30:00
       大小: 1.8M
       路径: /workspaces/AURO2025/solution/cartographer_states/state_20251229_163000.pbstream

请选择状态编号 [1-2] 或按 Enter 使用最新: 1

✅ 已选择: 我的地图_第一阶段

🚀 启动 Cartographer (加载状态)...
按 Enter 启动 Cartographer...
```

#### 步骤 3：启动 LLM 建图（继续探索）
```bash
# 终端 3
cd /workspaces/AURO2025 && source install/setup.bash
export OPENAI_API_KEY="sk-proj-你的密钥"
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

#### 步骤 4：继续建图后再次保存
```bash
# 终端 4
bash 保存Cartographer状态.sh robot1 我的地图_第二阶段
```

---

## 📋 脚本使用说明

### 保存脚本：`保存Cartographer状态.sh`

**基本用法：**
```bash
bash 保存Cartographer状态.sh [机器人命名空间] [状态名称]
```

**示例：**
```bash
# 使用默认命名（时间戳）
bash 保存Cartographer状态.sh

# 指定状态名称
bash 保存Cartographer状态.sh robot1 办公室东区

# 其他机器人
bash 保存Cartographer状态.sh robot2 实验室A区
```

**保存的文件：**
- `状态名称.pbstream` - SLAM 状态（主文件，用于继续建图）
- `状态名称_map.pgm/.yaml` - 地图图像（用于可视化）
- `状态名称_info.txt` - 元数据信息
- `resume_状态名称.sh` - 快速恢复脚本
- `latest.pbstream` - 最新状态的符号链接

---

### 加载脚本：`加载Cartographer状态.sh`

**基本用法：**
```bash
bash 加载Cartographer状态.sh [机器人命名空间] [状态文件路径]
```

**示例：**
```bash
# 交互式选择（推荐）
bash 加载Cartographer状态.sh

# 直接指定文件
bash 加载Cartographer状态.sh robot1 /workspaces/AURO2025/solution/cartographer_states/我的地图_第一阶段.pbstream

# 使用快速恢复脚本
bash /workspaces/AURO2025/solution/cartographer_states/resume_我的地图_第一阶段.sh
```

---

## 🔍 与之前脚本的区别

| 功能 | 旧脚本（地图加载） | 新脚本（Cartographer 状态） |
|------|------------------|---------------------------|
| 保存内容 | 地图图像 (.pgm) | SLAM 状态 (.pbstream) |
| 继续建图 | ❌ 不支持 | ✅ 完全支持 |
| 用途 | 静态地图可视化 | 增量建图、暂停恢复 |
| 服务调用 | map_saver | write_state |
| 加载方式 | map_server | cartographer (load_state_filename) |

---

## 💡 实际应用场景

### 场景 A：分时段建图
```bash
# 第一天：建图 1 小时
bash 保存Cartographer状态.sh robot1 第一天

# 第二天：继续建图
bash 加载Cartographer状态.sh robot1 .../第一天.pbstream
# ... 建图 1 小时 ...
bash 保存Cartographer状态.sh robot1 第二天

# 第三天：继续建图
bash 加载Cartographer状态.sh robot1 .../第二天.pbstream
```

### 场景 B：保险备份
```bash
# 每 30 分钟自动保存（在建图过程中）
while true; do
  sleep 1800
  bash 保存Cartographer状态.sh robot1 "auto_backup_$(date +%H%M)"
done
```

### 场景 C：多版本对比
```bash
# 保存不同探索策略的结果
bash 保存Cartographer状态.sh robot1 版本1_保守策略
# ... 调整参数 ...
bash 保存Cartographer状态.sh robot1 版本2_激进策略
# ... 对比效果 ...
```

---

## ⚠️  注意事项

### 1. Cartographer 服务必须运行
保存时需要 Cartographer 节点正在运行：
```bash
# 检查服务是否可用
ros2 service list | grep write_state
```

### 2. 仿真环境必须先启动
加载状态前必须先启动仿真环境（终端 1）。

### 3. 不要在建图过程中关闭 Cartographer
如果需要暂停：
1. 先保存状态
2. 再关闭所有节点
3. 下次从保存的状态恢复

### 4. 状态文件较大
`.pbstream` 文件会随着建图时间增长：
- 1 小时建图：约 2-5 MB
- 2 小时建图：约 5-10 MB

定期清理旧状态文件。

---

## 🔧 故障排查

### 问题 1：保存时提示 "write_state 服务不可用"
**原因**：Cartographer 未运行

**解决**：
```bash
# 检查 Cartographer 节点
ros2 node list | grep cartographer

# 重新启动 Cartographer
ros2 launch assessment cartographer.launch.py \
  ns:=robot1 use_sim_time:=true use_rviz:=true
```

---

### 问题 2：加载状态后地图是空的
**原因**：状态文件损坏或路径错误

**解决**：
```bash
# 检查文件是否存在且大小正常
ls -lh /workspaces/AURO2025/solution/cartographer_states/*.pbstream

# 尝试加载其他状态文件
bash 加载Cartographer状态.sh  # 交互式选择
```

---

### 问题 3：RViz 中看不到地图
**原因**：需要等待 Cartographer 加载完成

**解决**：
- 加载状态后等待 10-30 秒
- 检查 RViz 的 Fixed Frame 是否设置为 `map`
- 确认已添加 Map 显示项（话题：`/robot1/map`）

---

## 📊 文件位置

**保存目录**：`/workspaces/AURO2025/solution/cartographer_states/`

**文件结构**：
```
solution/cartographer_states/
├── 我的地图_第一阶段.pbstream           # SLAM 状态（主文件）
├── 我的地图_第一阶段_map.pgm            # 地图图像
├── 我的地图_第一阶段_map.yaml           # 地图配置
├── 我的地图_第一阶段_info.txt           # 元数据
├── resume_我的地图_第一阶段.sh          # 快速恢复脚本
└── latest.pbstream -> 我的地图_第一阶段.pbstream  # 最新状态链接
```

---

## 🎯 最佳实践

1. **定期保存**：建图过程中每 30-60 分钟保存一次
2. **描述性命名**：使用有意义的名称（如 "办公室东区" 而非 "map1"）
3. **保留关键节点**：重要阶段的状态永久保存
4. **清理旧文件**：定期删除测试和临时状态文件
5. **备份重要状态**：将 .pbstream 文件复制到安全位置

---

## 📚 相关资源

- [Cartographer 官方文档](https://google-cartographer-ros.readthedocs.io/)
- [LLM 建图参数说明](./LLM建图参数说明.md)
- [探索欲望与动态速度说明](./探索欲望与动态速度说明.md)

---

**文档版本**：v1.0
**最后更新**：2025-12-30
**状态**：✅ 生产可用
