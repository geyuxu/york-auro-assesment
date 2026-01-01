# AURO2025 Barrel Collection Robot - 项目成果总结

## 项目概述

基于 ROS2 Humble + Nav2 的自主桶收集机器人，在 Gazebo 仿真环境中完成红色污染桶的收集、运送到绿色区域、并在蓝色区域进行洗消的任务。

---

## 核心成果

### 1. 完整的状态机控制系统

实现了 10+ 状态的有限状态机：

```
IDLE → SEARCHING → SCANNING → APPROACHING → RAMMING →
NAVIGATING_TO_GREEN → DELIVERING → POST_DELIVER_REPOSITION →
NAVIGATING_TO_CYAN → DECONTAMINATING → (循环)
```

### 2. Nav2 导航集成

- AMCL 定位 + 预建地图
- 自定义 Nav2 参数配置
- 导航卡住检测与恢复
- 动态清除 costmap 避免幽灵障碍

### 3. 视觉伺服系统

- 基于像素偏移的视觉对准 (`x_offset → angular.z`)
- 视觉传感器检测红/蓝桶和绿/蓝区域
- 根据桶大小(size)判断距离

### 4. RAMMING 收桶流程

经过多次迭代优化的 4 阶段收桶策略：

| Phase | 动作 | 说明 |
|-------|------|------|
| 0 | 前进冲撞 | 推动桶 |
| 1 | 180° 原地旋转 | 基于角度检测，非时间 |
| 2 | 倒车对准 | 后方 LiDAR 验证桶位置 |
| 3 | 调用 pickup 服务 | 拾取桶 |

### 5. LiDAR 安全层

- 360° 激光雷达障碍检测
- 分区域距离监测 (front/left/right/rear/front_left/front_right)
- 动态 mask 过滤（携带桶时屏蔽后方）
- 紧急避障覆盖所有状态

### 6. 卡住检测与恢复

- 8秒/8cm 卡住检测阈值
- 只在导航状态触发，不打断收桶流程
- 多种恢复策略（转向开阔方向、倒车等）

---

## 关键优化记录

### RAMMING 阶段
- `d85ff83` 基于角度的 180° 旋转检测（替代时间检测）
- `00b30df` ramming 对准桶优化
- `9d86e35` 加快 ramming 旋转角度
- `241ef0a` ramming 阶段雷达验证背后桶

### APPROACHING 阶段
- `01eaa0e` approaching 卡墙脚处理
- `edafe05` approach timeout 逻辑优化
- `8735da1` 视觉导航+雷达避障

### 导航优化
- `04b1657` Nav2 + AMCL 定位系统
- `23b0741` nav2 searching 阶段避免撞墙
- `c107360` nav2 避障优化
- `61eb443` nav2 路径规划后直接导航
- `8396115` 导航卡住检测处理

### 其他优化
- `556b18b` 小车放下桶之前向空地位移
- `2572bfc` 调整 scanning 转向角度

---

## 代码架构

```
solution/solution/
├── cleaner_bot.py      # 主控制器 (~2800行)
│   ├── State Machine   # 状态机定义和转换
│   ├── Subscribers     # LiDAR, 视觉, 位置订阅
│   ├── Nav2 Client     # 导航目标发送
│   └── Service Clients # pickup/offload/decontaminate
│
├── llm_interface.py    # LLM 接口占位 (待实现)
└── data_logger.py      # 数据记录
```

---

## 运行方式

```bash
# 构建
colcon build --symlink-install

# 运行（单机器人）
ros2 launch solution solution_launch.py num_robots:=1 use_nav2:=true

# 运行指定场景
./rcutil.py run-scenario "Scenario 1"
```

---

## 当前性能

- 稳定收集 2-3 个桶（取决于随机种子）
- 能处理大部分卡住情况
- 视觉伺服对准成功率较高
- RAMMING 流程可靠性提升

---

## 已知限制

1. 桶位置随机，无法硬编码路径
2. 远距离桶检测不稳定
3. 多桶重叠时可能选错目标
4. 复杂角落可能卡住

---

## 实验性分支

`feature/llm-integration` 分支包含 LLM/Vision API 集成实验：
- OpenAI gpt-4o-mini 视觉导航
- 多级智能控制架构
- 状态：WIP，未合并

---

## 文件说明

| 文件 | 说明 |
|------|------|
| `cleaner_bot.py` | 主控制节点 |
| `llm_interface.py` | LLM 接口（占位） |
| `solution_launch.py` | 启动文件 |
| `custom_nav2_params_namespaced.yaml` | Nav2 参数 |
| `initial_poses.yaml` | 机器人初始位置 |

---

*Last updated: 2026-01-01*
