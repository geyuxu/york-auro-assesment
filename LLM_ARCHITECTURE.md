# LLM 多级智能控制架构

## 当前架构 (v1 - WIP)

```
┌─────────────────────────────────────────┐
│  LLM 策略层 (get_strategy_decision)     │  ← 高层决策 (3s rate limit)
│  - 决定：找桶/送桶/洗消/脱困             │
├─────────────────────────────────────────┤
│  Vision API 层 (vision_approach_barrel) │  ← 视觉导航 (4s rate limit)
│  - 图像分析：桶在哪、怎么走              │
├─────────────────────────────────────────┤
│  规则状态机 (State Machine)              │  ← 任务执行
│  - SEARCHING → APPROACHING → RAMMING    │
│  - 规则视觉伺服 (fallback)               │
├─────────────────────────────────────────┤
│  安全层 (LiDAR 硬阈值)                   │  ← 实时安全 (10Hz)
│  - 紧急避障 (front < 0.25m)             │
│  - 卡住检测 (8s/8cm)                    │
└─────────────────────────────────────────┘
```

## 关键设计原则

1. **降级机制** - LLM/Vision 不可用时 fallback 到规则
2. **安全层优先** - 避障永远覆盖高层决策
3. **异步处理** - API 调用不阻塞实时控制
4. **独立 Rate Limit** - Vision API 和 LLM 分开限流

## 当前实现状态

### 已完成
- [x] LLM 接口基础架构 (`llm_interface.py`)
- [x] Vision API 脱困 (`vision_drive`)
- [x] Vision API 接近桶 (`vision_approach_barrel`)
- [x] 直接 pickup 尝试 (`_try_direct_pickup`)
- [x] 失败降级到 RAMMING
- [x] 卡住检测只在导航状态触发

### 待完成
- [ ] 测试 Vision APPROACHING 效果
- [ ] 优化 Vision prompt（更精确的方向指令）
- [ ] 添加 Vision 送桶 (`vision_approach_zone`)
- [ ] LLM 策略层集成到状态机
- [ ] 多轮对话上下文（记住之前的决策）

## 业界参考架构

### 三层架构 (3T Architecture)
- Deliberative Layer → LLM 策略
- Sequencing Layer → 状态机
- Reactive Layer → 安全层

### 自动驾驶分层
- Perception → 视觉传感器
- Prediction → (暂无)
- Planning → LLM + 状态机
- Control → cmd_vel

### VLM 端到端 vs 混合架构
- **纯 VLM**：延迟高(1-2s)，不适合 10Hz 控制
- **混合架构**（当前）：VLM 做感知+决策，规则做执行+安全

## API 调用策略

```python
# LLM 策略决策
min_call_interval = 3.0s  # 节省 token

# Vision API
min_vision_call_interval = 4.0s  # 独立限流
model = "gpt-4o-mini"  # Vision 需要 4o 系列
detail = "low"  # 减少 token 消耗
timeout = 8.0s  # Vision 需要更长时间
```

## 下一步计划

### Phase 1: 验证 Vision APPROACHING
1. 测试当前代码效果
2. 观察 Vision API 返回的动作是否合理
3. 调整 prompt 和速度参数

### Phase 2: 扩展 Vision 能力
1. 添加 `vision_approach_zone` 用于送桶
2. 添加 `vision_escape` 用于复杂脱困场景

### Phase 3: LLM 策略集成
1. 使用 `get_strategy_decision` 覆盖状态切换
2. 添加事件历史上下文
3. 实现更智能的巡航路径规划

### Phase 4: 优化
1. 减少 API 调用次数
2. 添加本地 VLM fallback (如 LLaVA)
3. 缓存相似场景的决策

## 文件结构

```
solution/solution/
├── cleaner_bot.py      # 主控制器 + 状态机
├── llm_interface.py    # LLM/Vision API 封装
└── llm_mapper.py       # (未使用) LLM 建图
```

## Git 分支

- `feature/llm-integration` - LLM 集成开发分支
- 最新 commit: `484d949` - WIP: Vision-guided APPROACHING

---

*Last updated: 2026-01-01*
