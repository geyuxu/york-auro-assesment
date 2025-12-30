#!/usr/bin/env python3
"""
LLM 驱动的智能建图节点
使用大语言模型做决策，提高建图智能性
"""

import sys
import json
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

# 可选：如果安装了 openai 库
try:
    from openai import OpenAI
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False
    print("Warning: openai library not found. LLM features disabled.")
    print("Install with: pip install openai")


class MappingState(Enum):
    """建图状态"""
    IDLE = 0
    MOVING_FORWARD = 1
    ROTATING_LEFT = 2
    ROTATING_RIGHT = 3
    BACKING_UP = 4


class LLMMapper(Node):
    """LLM 驱动的智能建图节点"""

    def __init__(self):
        super().__init__('llm_mapper')

        self.get_logger().info("🤖 LLM 驱动的智能建图节点启动中...")

        # 运动参数 - 降低速度以提高 SLAM 稳定性
        self.LINEAR_SPEED = 0.08
        self.ANGULAR_SPEED = 0.25
        self.BACKUP_SPEED = -0.05

        # LLM 配置 - OpenAI API
        self.use_llm = HAS_OPENAI

        # 从环境变量读取配置
        import os
        self.api_key = os.getenv('OPENAI_API_KEY', '')
        self.api_base = os.getenv('OPENAI_API_BASE', 'https://api.openai.com/v1')
        self.llm_model = os.getenv('OPENAI_MODEL', 'gpt-4o-mini')  # 默认使用 gpt-4o-mini（快速且便宜）

        # 初始化 OpenAI 客户端
        if self.use_llm and self.api_key:
            self.llm_client = OpenAI(api_key=self.api_key, base_url=self.api_base)
        else:
            self.llm_client = None
            if self.use_llm:
                self.get_logger().warn("⚠️  未设置 OPENAI_API_KEY 环境变量")
                self.use_llm = False

        self.llm_decision_interval = 5.0  # 每 5 秒请求一次 LLM 决策
        self.last_llm_time = time.time()
        self.llm_failure_count = 0  # LLM 失败计数
        self.max_llm_failures = 5  # 连续失败 5 次后切换到规则

        # 状态
        self.state = MappingState.MOVING_FORWARD
        self.scan_data = None
        self.current_pose = None
        self.map_data = None  # 地图数据

        # 建图时间
        self.start_time = time.time()
        self.total_mapping_time = 7200  # 2h
        self.state_start_time = time.time()

        # 历史决策记录
        self.decision_history = []

        # 发布器和订阅器
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

        # 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)

        if self.use_llm:
            self.get_logger().info("✅ LLM 决策系统已启用")
            self.get_logger().info(f"   模型: {self.llm_model}")
            self.get_logger().info(f"   端点: {self.api_base}")
        else:
            self.get_logger().warn("⚠️  LLM 决策系统未启用（回退到规则决策）")

        self.get_logger().info("🚀 智能建图开始！")

    def scan_callback(self, msg):
        """处理激光雷达数据"""
        self.scan_data = msg

    def odom_callback(self, msg):
        """处理里程计数据"""
        self.current_pose = msg.pose.pose

    def map_callback(self, msg):
        """处理地图数据"""
        self.map_data = msg

    def get_map_summary(self):
        """获取地图摘要信息"""
        if self.map_data is None:
            return {
                'coverage': 0.0,
                'explored_cells': 0,
                'total_cells': 0,
                'unknown_ratio': 1.0
            }

        # 统计地图信息
        data = self.map_data.data
        total_cells = len(data)

        # OccupancyGrid 值：-1=未知, 0-100=已知（0=空闲, 100=占用）
        unknown_cells = sum(1 for cell in data if cell == -1)
        free_cells = sum(1 for cell in data if cell >= 0 and cell < 50)
        occupied_cells = sum(1 for cell in data if cell >= 50)
        explored_cells = total_cells - unknown_cells

        return {
            'coverage': (explored_cells / total_cells * 100) if total_cells > 0 else 0.0,
            'explored_cells': explored_cells,
            'total_cells': total_cells,
            'unknown_ratio': (unknown_cells / total_cells) if total_cells > 0 else 1.0,
            'free_cells': free_cells,
            'occupied_cells': occupied_cells
        }

    def evaluate_exploration_value(self):
        """评估各个方向的探索价值（未知区域密度）"""
        if self.scan_data is None or self.map_data is None:
            return {
                'front_unknown': 0.5,
                'left_unknown': 0.5,
                'right_unknown': 0.5
            }

        ranges = self.scan_data.ranges
        n = len(ranges)

        # 计算各方向未知区域比例（远距离传感器读数表明可能是未探索区域）
        def count_unknown_direction(indices):
            """统计某个方向的远距离点（可能是未知区域）"""
            far_points = 0
            valid_points = 0
            for i in indices:
                if i < len(ranges):
                    r = ranges[i]
                    if 0.1 < r < 10.0:
                        valid_points += 1
                        # 距离较远（>2m）可能是未探索区域
                        if r > 2.0:
                            far_points += 1
            return (far_points / valid_points) if valid_points > 0 else 0.0

        # 前方扇区
        front_indices = list(range(n - n//8, n)) + list(range(0, n//8))
        front_unknown = count_unknown_direction(front_indices)

        # 左侧扇区
        left_indices = range(n//4, 3*n//8)
        left_unknown = count_unknown_direction(left_indices)

        # 右侧扇区
        right_indices = range(5*n//8, 3*n//4)
        right_unknown = count_unknown_direction(right_indices)

        return {
            'front_unknown': front_unknown,
            'left_unknown': left_unknown,
            'right_unknown': right_unknown
        }

    def get_sensor_summary(self):
        """获取传感器数据摘要 - 考虑机器人尺寸的碰撞风险评估"""
        if self.scan_data is None:
            return {
                'front': float('inf'),
                'left': float('inf'),
                'right': float('inf'),
                'back': float('inf'),
                'front_collision_risk': False,
                'left_collision_risk': False,
                'right_collision_risk': False
            }

        # TurtleBot3 Waffle Pi 物理尺寸
        ROBOT_RADIUS = 0.22  # 机器人半径（米）
        SAFETY_MARGIN = 0.05  # 安全裕度（米）
        EFFECTIVE_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN  # 总安全半径 = 0.27m

        ranges = self.scan_data.ranges
        n = len(ranges)
        angle_increment = 2 * 3.14159 / n  # 激光雷达角度增量

        # 评估碰撞风险：检查障碍物是否在机器人路径上
        def evaluate_collision_risk(sector_ranges, sector_angles):
            """评估某个扇区的碰撞风险 - 多级评估"""
            min_safe_distance = float('inf')
            has_collision_risk = False
            critical_obstacles = 0  # 紧急障碍物计数

            for i, (distance, angle) in enumerate(zip(sector_ranges, sector_angles)):
                if 0.1 < distance < 10.0:
                    # 计算障碍物到机器人中心线的垂直距离
                    perpendicular_dist = abs(distance * math.sin(angle))

                    # 如果垂直距离小于机器人半径，说明在碰撞路径上
                    if perpendicular_dist < EFFECTIVE_RADIUS:
                        # 计算沿运动方向的距离
                        forward_dist = distance * math.cos(angle)
                        if forward_dist > 0:  # 只考虑前方的障碍物
                            min_safe_distance = min(min_safe_distance, forward_dist)

                            # 紧急碰撞风险：距离 < 0.35m
                            if forward_dist < 0.35:
                                critical_obstacles += 1
                                has_collision_risk = True
                            # 中等风险：距离 < 0.5m 且垂直距离很小（< 0.15m）
                            elif forward_dist < 0.5 and perpendicular_dist < 0.15:
                                has_collision_risk = True

            # 只有当多个紧急障碍物（>= 3个点）或确实很近时才标记为风险
            if critical_obstacles < 3 and min_safe_distance > 0.35:
                has_collision_risk = False

            return min_safe_distance, has_collision_risk

        # 前方扇区：-22.5° 到 +22.5°（45° 扇区）
        front_indices = list(range(n - n//8, n)) + list(range(0, n//8))
        front_ranges = [ranges[i] for i in front_indices]
        front_angles = [(i - n if i >= n - n//8 else i) * angle_increment for i in front_indices]
        front_min, front_risk = evaluate_collision_risk(front_ranges, front_angles)

        # 左侧扇区：67.5° 到 112.5°（45° 扇区）
        left_indices = range(n//4 + n//8, n//4 + 3*n//8)
        left_ranges = [ranges[i] for i in left_indices]
        left_angles = [(i * angle_increment - 1.5708) for i in left_indices]  # 相对于左侧90°
        left_min, left_risk = evaluate_collision_risk(left_ranges, left_angles)

        # 右侧扇区：-112.5° 到 -67.5°（45° 扇区）
        right_indices = range(5*n//8 + n//8, 3*n//4 + n//8)
        right_ranges = [ranges[i] for i in right_indices]
        right_angles = [(i * angle_increment - 4.7124) for i in right_indices]  # 相对于右侧-90°
        right_min, right_risk = evaluate_collision_risk(right_ranges, right_angles)

        # 后方扇区：简单统计（不评估碰撞风险）
        back_indices = range(3*n//8, 5*n//8)
        back_ranges = [r for i in back_indices if 0.1 < (r := ranges[i]) < 10.0]

        return {
            'front': front_min,
            'left': left_min,
            'right': right_min,
            'back': min(back_ranges) if back_ranges else float('inf'),
            'front_collision_risk': front_risk,
            'left_collision_risk': left_risk,
            'right_collision_risk': right_risk
        }

    def query_llm(self, prompt):
        """查询 LLM 获取决策"""
        if not self.use_llm or not self.llm_client:
            return None

        try:
            response = self.llm_client.chat.completions.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": "You are a robot navigation assistant. Reply with only ONE word: FORWARD, ROTATE_LEFT, ROTATE_RIGHT, or BACKUP."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=10,
                timeout=5.0
            )

            if response.choices and len(response.choices) > 0:
                return response.choices[0].message.content.strip()
            else:
                self.get_logger().warn("LLM 返回空响应")
                return None

        except Exception as e:
            self.get_logger().warn(f"LLM 查询错误: {e}")
            return None

    def get_llm_decision(self):
        """使用 LLM 做出导航决策 - 基于碰撞风险评估 + 探索欲望"""
        sensors = self.get_sensor_summary()
        map_summary = self.get_map_summary()
        exploration = self.evaluate_exploration_value()
        elapsed = time.time() - self.start_time

        # 获取距离和碰撞风险
        F, L, R = sensors['front'], sensors['left'], sensors['right']
        F_risk = sensors['front_collision_risk']
        L_risk = sensors['left_collision_risk']
        R_risk = sensors['right_collision_risk']

        # 获取探索价值
        F_explore = exploration['front_unknown']
        L_explore = exploration['left_unknown']
        R_explore = exploration['right_unknown']

        # 智能场景识别 - 基于实际碰撞风险而非简单距离
        if F_risk and L_risk and R_risk:
            # 三面都有碰撞风险 - 死胡同
            scenario = "⚠️  死胡同"
            action_hint = "BACKUP immediately, stuck"
        elif L_risk and not F_risk and F > 0.5:
            # 左侧有碰撞风险但前方安全 - 右转
            scenario = "➡️  左侧障碍"
            action_hint = "ROTATE_RIGHT to avoid left obstacle"
        elif R_risk and not F_risk and F > 0.5:
            # 右侧有碰撞风险但前方安全 - 左转
            scenario = "⬅️  右侧障碍"
            action_hint = "ROTATE_LEFT to avoid right obstacle"
        elif L < 0.8 and R < 0.8 and not F_risk and F > 0.5:
            # 两侧都近（狭窄通道）但前方无碰撞风险 - 前进
            scenario = "🚪 狭窄通道"
            action_hint = "FORWARD through narrow passage"
        elif not F_risk and F > 0.6:
            # 前方无碰撞风险且开阔 - 安全前进
            scenario = "🛣️  前方开阔"
            action_hint = "FORWARD safely"
        elif F_risk and F < 0.4:
            # 前方有紧急碰撞风险 - 后退
            scenario = "🚨 前方过近"
            action_hint = "BACKUP to avoid collision"
        else:
            # 需要转向避障或探索 - 优先选择未探索区域多的方向
            scenario = "🔄 需要转向"
            # 综合考虑开阔度和探索价值
            left_score = L * 0.5 + L_explore * 2.0
            right_score = R * 0.5 + R_explore * 2.0
            if left_score > right_score:
                action_hint = f"ROTATE_LEFT toward unexplored (L_explore={L_explore:.2f})"
            else:
                action_hint = f"ROTATE_RIGHT toward unexplored (R_explore={R_explore:.2f})"

        # 生成 LLM 提示词 - 强调碰撞风险评估和探索欲望
        risk_status = f"F_risk={F_risk} L_risk={L_risk} R_risk={R_risk}"
        explore_status = f"F_unknown={F_explore:.2f} L_unknown={L_explore:.2f} R_unknown={R_explore:.2f}"
        prompt = f"""Robot mapper (SLAM). Robot radius: 0.27m (including safety margin).
Distances: F={F:.2f}m L={L:.2f}m R={R:.2f}m
Collision Risk: {risk_status}
Exploration Value: {explore_status}
Map: {map_summary['coverage']:.1f}% explored, {map_summary['unknown_ratio']*100:.1f}% unknown
Time: {elapsed:.0f}s/{self.total_mapping_time}s
Scenario: {scenario}

Intelligent Rules (considering robot size + exploration desire):
1. F_risk && L_risk && R_risk → BACKUP (stuck)
2. L_risk && !F_risk && F>0.5m → ROTATE_RIGHT (left blocked)
3. R_risk && !F_risk && F>0.5m → ROTATE_LEFT (right blocked)
4. L<0.8m && R<0.8m && !F_risk && F>0.5m → FORWARD (narrow passage safe)
5. !F_risk && F>0.6m → FORWARD (open path)
6. F_risk && F<0.4m → BACKUP (emergency)
7. Otherwise → Rotate toward MORE UNEXPLORED side (higher unknown value)

EXPLORATION PRIORITY: Prefer directions with higher unknown values (unexplored areas).

Suggestion: {action_hint}

CRITICAL: Collision risk considers robot's 0.27m radius. Only avoid obstacles that actually block the path!
Choose ONE word:"""

        response = self.query_llm(prompt)

        if response:
            # 解析 LLM 响应
            response = response.upper().strip()

            # 提取第一个单词
            first_word = response.split()[0] if response.split() else response

            if 'FORWARD' in first_word or 'F' == first_word:
                return MappingState.MOVING_FORWARD
            elif 'LEFT' in first_word or 'L' in first_word or 'ROTATE_LEFT' in response:
                return MappingState.ROTATING_LEFT
            elif 'RIGHT' in first_word or 'R' in first_word or 'ROTATE_RIGHT' in response:
                return MappingState.ROTATING_RIGHT
            elif 'BACK' in first_word or 'B' in first_word:
                return MappingState.BACKING_UP

        return None

    def rule_based_decision(self):
        """基于规则的决策（LLM 不可用时的回退）- 使用碰撞风险评估 + 探索欲望"""
        sensors = self.get_sensor_summary()
        exploration = self.evaluate_exploration_value()

        F, L, R = sensors['front'], sensors['left'], sensors['right']
        F_risk = sensors['front_collision_risk']
        L_risk = sensors['left_collision_risk']
        R_risk = sensors['right_collision_risk']

        L_explore = exploration['left_unknown']
        R_explore = exploration['right_unknown']

        # 智能规则决策树 - 基于碰撞风险 + 探索欲望
        # 规则 1: 三面都有碰撞风险 - 后退
        if F_risk and L_risk and R_risk:
            return MappingState.BACKING_UP

        # 规则 2: 左侧有风险但前方安全 - 右转
        if L_risk and not F_risk and F > 0.5:
            return MappingState.ROTATING_RIGHT

        # 规则 3: 右侧有风险但前方安全 - 左转
        if R_risk and not F_risk and F > 0.5:
            return MappingState.ROTATING_LEFT

        # 规则 4: 狭窄通道但前方无碰撞风险 - 前进
        if L < 0.8 and R < 0.8 and not F_risk and F > 0.5:
            return MappingState.MOVING_FORWARD

        # 规则 5: 前方无碰撞风险且开阔 - 前进
        if not F_risk and F > 0.6:
            return MappingState.MOVING_FORWARD

        # 规则 6: 前方有紧急碰撞风险 - 后退
        if F_risk and F < 0.4:
            return MappingState.BACKING_UP

        # 规则 7: 默认 - 转向未探索区域更多的一侧
        # 综合开阔度（50%权重）和探索价值（200%权重，优先探索）
        left_score = L * 0.5 + L_explore * 2.0
        right_score = R * 0.5 + R_explore * 2.0

        if left_score > right_score:
            return MappingState.ROTATING_LEFT
        else:
            return MappingState.ROTATING_RIGHT

    def execute_action(self, state):
        """执行运动指令 - 动态速度调整"""
        twist = Twist()

        # 获取碰撞风险评估
        sensors = self.get_sensor_summary()
        F_risk = sensors['front_collision_risk']
        L_risk = sensors['left_collision_risk']
        R_risk = sensors['right_collision_risk']
        F = sensors['front']

        # 计算风险等级
        total_risk = sum([F_risk, L_risk, R_risk])  # 0-3

        # 动态速度调整 - 优化狭窄空间通行
        if state == MappingState.MOVING_FORWARD:
            # 根据风险和前方距离调整速度
            if F_risk and F < 0.35:
                # 紧急刹停：前方非常近且有碰撞风险
                twist.linear.x = 0.0
            elif F < 0.5:
                # 狭窄空间缓慢前进：即使没有碰撞风险，也要小心
                twist.linear.x = self.LINEAR_SPEED * 0.3  # 极慢速度 0.024 m/s
            elif F < 0.8 or total_risk >= 2:
                # 中等风险：减速
                twist.linear.x = self.LINEAR_SPEED * 0.6  # 减速40%
            elif total_risk == 1 or F < 1.5:
                # 轻度风险：正常速度
                twist.linear.x = self.LINEAR_SPEED
            else:
                # 低风险且开阔：加速
                twist.linear.x = self.LINEAR_SPEED * 1.5  # 加速50%，最高0.12m/s

        elif state == MappingState.ROTATING_LEFT:
            # 旋转速度根据周围风险调整
            if total_risk >= 2:
                twist.angular.z = self.ANGULAR_SPEED * 0.7  # 减速旋转
            else:
                twist.angular.z = self.ANGULAR_SPEED

        elif state == MappingState.ROTATING_RIGHT:
            # 旋转速度根据周围风险调整
            if total_risk >= 2:
                twist.angular.z = -self.ANGULAR_SPEED * 0.7  # 减速旋转
            else:
                twist.angular.z = -self.ANGULAR_SPEED

        elif state == MappingState.BACKING_UP:
            twist.linear.x = self.BACKUP_SPEED

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        """主控制循环"""
        elapsed_time = time.time() - self.start_time

        # 检查是否完成
        if elapsed_time > self.total_mapping_time:
            self.get_logger().info("🎉 建图时间已到！")
            self.get_logger().info("请保存地图：ros2 run nav2_map_server map_saver_cli -f solution/config/map2 --ros-args -r map:=/robot1/map")
            self.stop_robot()
            rclpy.shutdown()
            return

        # 进度更新（包含地图覆盖率）
        if int(elapsed_time) % 30 == 0 and int(elapsed_time) > 0:
            progress = (elapsed_time / self.total_mapping_time) * 100
            map_summary = self.get_map_summary()
            self.get_logger().info(
                f"📊 建图进度: {progress:.1f}% ({int(elapsed_time)}/{self.total_mapping_time}秒) | "
                f"地图覆盖: {map_summary['coverage']:.1f}%"
            )

        # LLM 决策
        current_time = time.time()
        if current_time - self.last_llm_time >= self.llm_decision_interval:
            self.last_llm_time = current_time

            # 尝试 LLM 决策
            if self.use_llm and self.llm_failure_count < self.max_llm_failures:
                new_state = self.get_llm_decision()
                if new_state:
                    # 获取传感器数据和碰撞风险
                    sensors = self.get_sensor_summary()
                    F, L, R = sensors['front'], sensors['left'], sensors['right']
                    F_risk = sensors['front_collision_risk']
                    L_risk = sensors['left_collision_risk']
                    R_risk = sensors['right_collision_risk']

                    # 场景识别（与决策逻辑一致）- 基于碰撞风险
                    if F_risk and L_risk and R_risk:
                        scenario = "⚠️  死胡同"
                    elif L_risk and not F_risk and F > 0.5:
                        scenario = "➡️  左侧障碍"
                    elif R_risk and not F_risk and F > 0.5:
                        scenario = "⬅️  右侧障碍"
                    elif L < 0.8 and R < 0.8 and not F_risk and F > 0.5:
                        scenario = "🚪 狭窄通道"
                    elif not F_risk and F > 0.6:
                        scenario = "🛣️  前方开阔"
                    elif F_risk and F < 0.4:
                        scenario = "🚨 前方过近"
                    else:
                        scenario = "🔄 需要转向"

                    # 生成碰撞风险状态字符串
                    risk_str = f"{'F' if F_risk else '_'}{'L' if L_risk else '_'}{'R' if R_risk else '_'}"

                    # 获取探索价值
                    exploration = self.evaluate_exploration_value()
                    max_explore = max(exploration['front_unknown'],
                                     exploration['left_unknown'],
                                     exploration['right_unknown'])

                    self.state = new_state
                    self.state_start_time = current_time
                    self.llm_failure_count = 0  # 重置失败计数
                    self.decision_history.append({
                        'time': elapsed_time,
                        'state': new_state.name,
                        'source': 'LLM'
                    })
                    self.get_logger().info(
                        f"🤖 LLM: {new_state.name:16s} | {scenario:12s} | "
                        f"F={F:.2f} L={L:.2f} R={R:.2f} | Risk:{risk_str} | Explore:{max_explore:.2f}"
                    )
                else:
                    # LLM 失败，增加计数并使用规则
                    self.llm_failure_count += 1
                    new_state = self.rule_based_decision()
                    self.state = new_state
                    self.state_start_time = current_time
                    if self.llm_failure_count >= self.max_llm_failures:
                        self.get_logger().warn(f"⚠️  LLM 连续失败 {self.max_llm_failures} 次，切换到规则决策")
                    else:
                        self.get_logger().debug(f"📋 规则决策 (LLM失败 {self.llm_failure_count}/{self.max_llm_failures}): {new_state.name}")
            else:
                # 没有 LLM 或失败太多次，使用规则
                new_state = self.rule_based_decision()
                self.state = new_state
                self.state_start_time = current_time

        # 执行当前动作
        self.execute_action(self.state)

        # 状态超时保护
        state_duration = time.time() - self.state_start_time
        if state_duration > 5.0:  # 5 秒后强制重新决策
            self.last_llm_time = 0  # 触发立即决策

    def destroy_node(self):
        self.stop_robot()

        # 输出决策统计
        if self.decision_history:
            llm_count = sum(1 for d in self.decision_history if d['source'] == 'LLM')
            total_count = len(self.decision_history)
            self.get_logger().info(f"📊 决策统计: {llm_count}/{total_count} 来自 LLM")

        self.get_logger().info("👋 智能建图节点已停止")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = LLMMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到停止信号")
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
