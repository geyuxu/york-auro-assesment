# 地图保存与加载脚本 - 快速参考

## 🎯 两个脚本，一套完整方案

### 📥 保存脚本：`保存地图和位置.sh`
保存当前建图进度，包括地图和机器人位置。

**快速使用**：
```bash
# 最简单（使用默认命名）
bash 保存地图和位置.sh

# 指定名称
bash 保存地图和位置.sh robot1 我的办公室地图
```

**保存内容**：
- ✅ 地图图像 (`.pgm`)
- ✅ 地图配置 (`.yaml`)
- ✅ 机器人位置 (`_pose.yaml`)
- ✅ 摘要信息 (`_summary.txt`)
- ✅ 快速加载脚本 (`load_*.sh`)

---

### 📤 加载脚本：`加载地图和位置.sh`
恢复之前保存的地图和机器人位置。

**快速使用**：
```bash
# 交互式选择（推荐）
bash 加载地图和位置.sh

# 直接指定
bash 加载地图和位置.sh robot1 /workspaces/AURO2025/solution/saved_maps/map_20251230_143000
```

**加载内容**：
- ✅ 启动地图服务器
- ✅ 发布地图到 `/robot1/map`
- ✅ 设置初始位置到 `/robot1/initialpose`

---

## 📋 典型工作流程

### 场景 1：建图中断恢复
```bash
# 1. 建图过程中保存进度
bash 保存地图和位置.sh

# 2. 关闭仿真或重启系统
# ...

# 3. 重新启动后加载
bash 加载地图和位置.sh
# 选择刚才保存的地图

# 4. 继续建图
ros2 launch solution solution_launch.py
```

---

### 场景 2：分区域建图
```bash
# 1. 建图区域 A
ros2 launch solution solution_launch.py
# ... 探索一段时间 ...
bash 保存地图和位置.sh robot1 区域A

# 2. 建图区域 B
# ... 移动到新区域 ...
bash 保存地图和位置.sh robot1 区域B

# 3. 稍后选择性加载
bash 加载地图和位置.sh  # 选择区域A或区域B
```

---

### 场景 3：对比不同版本
```bash
# 保存多个版本
bash 保存地图和位置.sh robot1 版本1_初步探索
bash 保存地图和位置.sh robot1 版本2_增加细节
bash 保存地图和位置.sh robot1 版本3_完整地图

# 加载任意版本对比
bash 加载地图和位置.sh  # 选择要查看的版本
```

---

## 🔍 文件位置

**保存目录**：`/workspaces/AURO2025/solution/saved_maps/`

**文件命名规则**：
```
map_20251230_143000.pgm           # 地图图像
map_20251230_143000.yaml          # 地图配置
map_20251230_143000_pose.yaml     # 位置信息
map_20251230_143000_summary.txt   # 摘要
load_map_20251230_143000.sh       # 快速加载脚本
```

---

## 🛠️ 前置条件

### 保存脚本需要：
1. ✅ ROS2 环境已 source (`source install/setup.bash`)
2. ✅ SLAM 节点正在运行（如 Cartographer）
3. ✅ 机器人仿真/硬件正在运行
4. ✅ `/robot1/map` 话题存在
5. ✅ `/robot1/odom` 或 TF 可用

### 加载脚本需要：
1. ✅ ROS2 环境已 source
2. ✅ 已保存的地图文件存在
3. ⚠️ 会停止已有的地图服务器

---

## ⚡ 快捷方式

每次保存都会自动生成快速加载脚本，可直接运行：

```bash
# 自动生成的快速加载脚本
bash /workspaces/AURO2025/solution/saved_maps/load_map_20251230_143000.sh
```

---

## 📊 查看保存的地图列表

```bash
ls -lht /workspaces/AURO2025/solution/saved_maps/
```

---

## 📖 完整文档

详细使用说明请参考：[地图保存与加载使用指南.md](./地图保存与加载使用指南.md)

---

## ❓ 常见问题

### Q1: 保存时提示 "未找到 ros2 命令"
**A**: 先执行 `source install/setup.bash`

### Q2: 保存时提示 "地图保存失败"
**A**: 确保 SLAM 节点（Cartographer）正在运行，检查 `ros2 topic list | grep map`

### Q3: 加载后机器人位置没变
**A**: 如果使用 AMCL 定位，确保 AMCL 节点正在运行

### Q4: 如何删除旧地图
**A**:
```bash
# 删除特定地图（所有相关文件）
rm /workspaces/AURO2025/solution/saved_maps/map_20251230_143000.*

# 保留最近 5 个，删除其他
cd /workspaces/AURO2025/solution/saved_maps/
ls -t map_*.yaml | grep -v "_pose.yaml" | tail -n +6 | xargs -I {} rm -f {%.yaml}*
```

---

## 🎯 提示

- 💡 建图过程中建议每 10-15 分钟保存一次
- 💡 使用描述性名称（如 `办公室东侧` 而不是 `map1`）
- 💡 重要地图建议创建版本备份
- 💡 定期清理不需要的旧地图文件

---

**快速参考版本**：v1.0
**完整文档**：地图保存与加载使用指南.md
**最后更新**：2025-12-30
