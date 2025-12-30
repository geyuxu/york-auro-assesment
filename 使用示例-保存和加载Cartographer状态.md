# Cartographer 状态保存和加载 - 完整使用示例

## 🎯 完整流程演示

### 场景：2小时建图，保存状态，第二天继续

---

## 第一天：建图 2 小时并保存

### 步骤 1：启动仿真环境

**终端 1**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

ros2 launch assessment assessment.launch.py \
  num_robots:=1 \
  use_nav2:=false \
  headless:=false \
  random_seed:=42
```

**预期输出**：
```
[INFO] [gazebo-1]: process started with pid [12345]
[INFO] [robot_state_publisher-2]: process started with pid [12346]
...
```

**等待**：Gazebo 完全启动（看到机器人出现）

---

### 步骤 2：启动 Cartographer SLAM

**终端 2**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

ros2 launch assessment cartographer.launch.py \
  ns:=robot1 \
  use_sim_time:=true \
  use_rviz:=true
```

**预期输出**：
```
[INFO] [cartographer_node-1]: process started with pid [12350]
[INFO] [cartographer_occupancy_grid_node-2]: process started with pid [12351]
[INFO] [rviz2-3]: process started with pid [12352]
...
```

**验证**：
- RViz 窗口打开
- 在 RViz 中看到激光扫描数据
- 地图开始逐渐显示

---

### 步骤 3：启动 LLM 建图节点

**终端 3**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

# 设置 OpenAI API 密钥
export OPENAI_API_KEY="sk-proj-你的完整密钥"

# 启动 LLM 建图
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

**预期输出**：
```
[INFO] [llm_mapper]: 🚀 智能建图开始！
[INFO] [llm_mapper]: 📊 建图参数：
[INFO] [llm_mapper]:    总时长: 7200 秒 (2.00 小时)
[INFO] [llm_mapper]:    线速度: 0.08 m/s
[INFO] [llm_mapper]:    角速度: 0.3 rad/s
...
🤖 LLM: MOVING_FORWARD   | 🛣️  前方开阔  | F=2.50 L=1.20 R=1.50
🤖 LLM: ROTATING_LEFT    | ⬅️  右墙左通  | F=0.80 L=2.00 R=0.40
...
```

**观察**：
- 机器人开始自主探索
- RViz 中地图逐渐扩大
- 终端 3 显示决策日志

---

### 步骤 4：建图进行中（1.5 小时后）

机器人已经探索了大部分区域，现在想保存进度。

**终端 4**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

# 保存当前状态
bash 保存Cartographer状态.sh robot1 第一天_下午建图
```

**预期输出**：
```
💾 保存 Cartographer SLAM 状态
================================

📋 配置信息：
  机器人命名空间: /robot1
  保存目录: /workspaces/AURO2025/solution/cartographer_states
  状态名称: 第一天_下午建图

🔍 检查 ROS2 运行环境...
✅ ROS2 环境正常

🔍 检查 Cartographer 服务...
✅ Cartographer 服务已找到

💾 步骤 1/3: 保存 Cartographer 状态...
✅ Cartographer 状态保存成功: /workspaces/AURO2025/solution/cartographer_states/第一天_下午建图.pbstream
   文件大小: 3.2M

💾 步骤 2/3: 保存地图图像（用于可视化）...
✅ 地图图像保存成功: 第一天_下午建图_map.pgm/.yaml

💾 步骤 3/3: 保存元数据...
✅ 元数据保存成功: 第一天_下午建图_info.txt
✅ 快速恢复脚本创建成功: resume_第一天_下午建图.sh
✅ 已更新最新状态链接: latest.pbstream

🎉 完成！Cartographer 状态已保存
```

**✅ 保存成功！** 现在可以安全关闭所有程序。

---

### 步骤 5：继续建图或关闭

**选项 A：继续建图到 2 小时**
- 让终端 3 的 LLM 建图继续运行
- 2 小时后会自动停止
- 再次保存最终状态

**选项 B：现在结束**
- 在终端 3 按 `Ctrl+C` 停止 LLM 建图
- 在终端 2 按 `Ctrl+C` 停止 Cartographer
- 在终端 1 按 `Ctrl+C` 停止仿真
- 状态已保存，可以随时恢复

---

### 步骤 6：2 小时后保存最终状态

如果让建图完成 2 小时：

**终端 4**：
```bash
bash 保存Cartographer状态.sh robot1 第一天_完整地图
```

**查看保存的文件**：
```bash
ls -lh /workspaces/AURO2025/solution/cartographer_states/
```

**输出示例**：
```
-rw-r--r-- 1 ubuntu ubuntu 3.2M Dec 30 14:30 第一天_下午建图.pbstream
-rw-r--r-- 1 ubuntu ubuntu 256K Dec 30 14:30 第一天_下午建图_map.pgm
-rw-r--r-- 1 ubuntu ubuntu  185 Dec 30 14:30 第一天_下午建图_map.yaml
-rw-r--r-- 1 ubuntu ubuntu 1.2K Dec 30 14:30 第一天_下午建图_info.txt
-rwxr-xr-x 1 ubuntu ubuntu  856 Dec 30 14:30 resume_第一天_下午建图.sh

-rw-r--r-- 1 ubuntu ubuntu 5.8M Dec 30 16:30 第一天_完整地图.pbstream
-rw-r--r-- 1 ubuntu ubuntu 512K Dec 30 16:30 第一天_完整地图_map.pgm
-rw-r--r-- 1 ubuntu ubuntu  185 Dec 30 16:30 第一天_完整地图_map.yaml
-rw-r--r-- 1 ubuntu ubuntu 1.2K Dec 30 16:30 第一天_完整地图_info.txt
-rwxr-xr-x 1 ubuntu ubuntu  856 Dec 30 16:30 resume_第一天_完整地图.sh

lrwxrwxrwx 1 ubuntu ubuntu   32 Dec 30 16:30 latest.pbstream -> 第一天_完整地图.pbstream
```

---

## 第二天：从保存的状态继续建图

### 步骤 1：启动仿真环境（同第一天）

**终端 1**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

ros2 launch assessment assessment.launch.py \
  num_robots:=1 \
  use_nav2:=false \
  headless:=false \
  random_seed:=42
```

**等待**：Gazebo 完全启动

---

### 步骤 2：加载 Cartographer 状态

**终端 2**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

bash 加载Cartographer状态.sh
```

**交互式界面**：
```
🔄 加载 Cartographer SLAM 状态
================================

📂 可用的 SLAM 状态：

   [1] 第一天_完整地图
       时间: 2025-12-30 16:30:00
       大小: 5.8M
       路径: /workspaces/AURO2025/solution/cartographer_states/第一天_完整地图.pbstream

   [2] 第一天_下午建图
       时间: 2025-12-30 14:30:00
       大小: 3.2M
       路径: /workspaces/AURO2025/solution/cartographer_states/第一天_下午建图.pbstream

请选择状态编号 [1-2] 或按 Enter 使用最新: 1

✅ 已选择: 第一天_完整地图

📋 加载配置：
  机器人命名空间: /robot1
  状态文件: /workspaces/AURO2025/solution/cartographer_states/第一天_完整地图.pbstream

🔍 检查仿真环境...
✅ 仿真环境已运行

🚀 启动 Cartographer (加载状态)...

⚠️  重要提示：
  1. Cartographer 将在此终端运行（不要关闭）
  2. RViz 会自动打开
  3. 启动完成后，在新终端运行 LLM 建图节点

💡 下一步：
  在新终端执行:
    cd /workspaces/AURO2025 && source install/setup.bash
    export OPENAI_API_KEY="你的API密钥"
    ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1

按 Enter 启动 Cartographer...
```

按 Enter 后：

```
[INFO] [cartographer_node-1]: process started with pid [23456]
[INFO] Loading state from /workspaces/AURO2025/solution/cartographer_states/第一天_完整地图.pbstream
[INFO] Loaded 128 submaps
[INFO] Loaded trajectory with 2456 nodes
[INFO] [cartographer_occupancy_grid_node-2]: process started with pid [23457]
[INFO] [rviz2-3]: process started with pid [23458]
```

**验证**：
- RViz 打开并立即显示之前的完整地图
- 机器人在地图上的历史轨迹可见
- 可以继续添加新数据

---

### 步骤 3：继续 LLM 建图

**终端 3**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

export OPENAI_API_KEY="sk-proj-你的完整密钥"

ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1
```

**预期输出**：
```
[INFO] [llm_mapper]: 🚀 智能建图开始！
...
🤖 LLM: MOVING_FORWARD   | 🛣️  前方开阔  | F=2.50 L=1.20 R=1.50
```

**观察**：
- 机器人从当前位置开始探索
- 新的扫描数据添加到已有地图
- 地图覆盖率继续增长

---

### 步骤 4：探索新区域后再次保存

**终端 4**：
```bash
bash 保存Cartographer状态.sh robot1 第二天_扩展地图
```

---

## 🔍 验证和检查

### 查看保存的状态信息

```bash
cat /workspaces/AURO2025/solution/cartographer_states/第一天_完整地图_info.txt
```

**输出**：
```
Cartographer SLAM 状态保存信息
=====================================

保存时间: Mon Dec 30 16:30:00 2025
机器人命名空间: robot1
状态名称: 第一天_完整地图

文件列表:
---------
1. SLAM 状态: 第一天_完整地图.pbstream (主文件)
2. 地图图像: 第一天_完整地图_map.pgm
3. 地图配置: 第一天_完整地图_map.yaml
4. 元数据: 第一天_完整地图_info.txt

文件大小:
---------
-rw-r--r-- 1 ubuntu ubuntu 5.8M Dec 30 16:30 第一天_完整地图.pbstream
...

如何继续建图:
-----------
# 终端 1 - 仿真
ros2 launch assessment assessment.launch.py \
  num_robots:=1 use_nav2:=false headless:=false random_seed:=42

# 终端 2 - 加载 SLAM 状态并继续建图
bash 加载Cartographer状态.sh robot1 /workspaces/AURO2025/solution/cartographer_states/第一天_完整地图.pbstream

# 终端 3 - LLM 建图
cd /workspaces/AURO2025 && source install/setup.bash
export OPENAI_API_KEY="你的API密钥"
ros2 run solution llm_mapper --ros-args --remap __ns:=/robot1

或者使用快速加载脚本:
bash /workspaces/AURO2025/solution/cartographer_states/resume_第一天_完整地图.sh
```

---

### 使用快速恢复脚本（替代方法）

**终端 1**：启动仿真（同上）

**终端 2**：
```bash
cd /workspaces/AURO2025
source install/setup.bash

# 使用自动生成的快速恢复脚本
bash /workspaces/AURO2025/solution/cartographer_states/resume_第一天_完整地图.sh
```

这个脚本会自动：
1. 提示确认仿真已启动
2. 启动 Cartographer 并加载指定状态
3. 启动 RViz

---

## ⚠️  常见问题

### Q1: 保存时提示 "ROS2 运行环境未启动"

**原因**：还没有启动仿真和 Cartographer

**解决**：按照终端 1、2、3 的顺序依次启动

---

### Q2: 加载状态后 RViz 显示空白地图

**原因**：Cartographer 正在加载状态，需要等待

**解决**：等待 10-30 秒，观察终端 2 的日志，看到 "Loaded X submaps" 后地图会出现

---

### Q3: 继续建图时机器人行为异常

**原因**：机器人在 Gazebo 中的位置与保存时不同

**解决**：
- 使用相同的 `random_seed` 确保环境一致
- 或者手动在 Gazebo 中将机器人移动到合适位置

---

### Q4: 状态文件越来越大

**原因**：Cartographer 保存了所有历史轨迹和 submap

**解决**：
- 这是正常的，2小时建图通常 5-10 MB
- 定期清理不需要的旧状态文件
- 保留关键节点的状态即可

---

## 💡 最佳实践

1. **定期保存**：每 30-60 分钟保存一次，避免进度丢失
2. **描述性命名**：使用有意义的名称（"办公室东区" 而非 "test1"）
3. **保留里程碑**：关键阶段的状态永久保存
4. **测试恢复**：保存后立即测试能否正确加载
5. **备份重要文件**：将 .pbstream 文件复制到安全位置

---

**完整流程图**：

```
第一天
├── 启动仿真 (终端1)
├── 启动 Cartographer (终端2)
├── 启动 LLM 建图 (终端3)
├── 建图 1.5 小时
├── 保存中间状态 (终端4) ← 第一次保存
├── 继续建图 0.5 小时
├── 保存最终状态 (终端4) ← 第二次保存
└── 关闭所有程序

第二天
├── 启动仿真 (终端1)
├── 加载状态 (终端2) ← 选择"第一天_完整地图"
├── 启动 LLM 建图 (终端3)
├── 继续探索新区域
├── 保存扩展地图 (终端4)
└── 完成
```

---

**文档版本**：v1.0
**最后更新**：2025-12-30
**状态**：✅ 实战验证
