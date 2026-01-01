"""
LLM Interface for Cleaner Bot

This module provides an interface for LLM-based decision making.
Uses OpenAI API (or compatible API) for intelligent decision support.

架构设计：
1. LLM 控制上层策略决策（找桶/送桶/洗消）
2. Vision API 控制底层驾驶（卡住时用图像分析）
3. 规则层做实时控制（简单情况）
"""

import os
import json
import time
import base64
from typing import Optional, List, Tuple, Dict, Any

# Optional: OpenAI library
try:
    from openai import OpenAI
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False
    print("Warning: openai library not found. LLM features disabled.")
    print("Install with: pip install openai")


class LLMInterface:
    """LLM interface for robot decision making."""

    def __init__(self, logger=None):
        """Initialize LLM interface.

        Args:
            logger: ROS2 logger for output (optional)
        """
        self.logger = logger
        self.enabled = HAS_OPENAI
        self.client = None

        # Configuration from environment variables
        self.api_key = os.getenv('OPENAI_API_KEY', '')
        self.api_base = os.getenv('OPENAI_API_BASE', 'https://api.openai.com/v1')
        self.model = os.getenv('OPENAI_MODEL', 'gpt-4o-mini')  # 最轻量的模型

        # Rate limiting - 控制API调用频率
        self.last_call_time = 0
        self.min_call_interval = 3.0  # 3秒最小间隔，节省API调用

        # Vision API 独立的 rate limit
        self.last_vision_call_time = 0
        self.min_vision_call_interval = 4.0  # Vision API 4秒间隔

        # Failure tracking
        self.failure_count = 0
        self.max_failures = 5  # Disable LLM after 5 consecutive failures

        # Event history for context
        self.event_history: List[Dict[str, Any]] = []
        self.max_history = 20

        # Initialize OpenAI client
        if self.enabled and self.api_key:
            try:
                self.client = OpenAI(api_key=self.api_key, base_url=self.api_base)
                self._log_info(f"LLM interface initialized (model: {self.model})")
            except Exception as e:
                self._log_warn(f"Failed to initialize OpenAI: {e}")
                self.enabled = False
        else:
            if self.enabled:
                self._log_warn("OPENAI_API_KEY not set, LLM disabled")
            self.enabled = False

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(f"[LLM] {msg}")
        else:
            print(f"[LLM] {msg}")

    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(f"[LLM] {msg}")
        else:
            print(f"[LLM WARNING] {msg}")

    def _rate_limit_check(self) -> bool:
        """Check if we should wait before making another API call."""
        now = time.time()
        if now - self.last_call_time < self.min_call_interval:
            return False
        self.last_call_time = now
        return True

    def _query_llm(self, system_prompt: str, user_prompt: str, max_tokens: int = 100) -> Optional[str]:
        """Query the LLM with rate limiting and error handling.

        Args:
            system_prompt: System message for the LLM
            user_prompt: User message for the LLM
            max_tokens: Maximum tokens in response

        Returns:
            Response text or None if failed
        """
        if not self.enabled or not self.client:
            return None

        if self.failure_count >= self.max_failures:
            return None

        if not self._rate_limit_check():
            return None

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=max_tokens,
                timeout=5.0
            )

            if response.choices and len(response.choices) > 0:
                self.failure_count = 0  # Reset on success
                return response.choices[0].message.content.strip()
            else:
                self._log_warn("LLM returned empty response")
                self.failure_count += 1
                return None

        except Exception as e:
            self._log_warn(f"LLM query error: {e}")
            self.failure_count += 1
            if self.failure_count >= self.max_failures:
                self._log_warn(f"LLM disabled after {self.max_failures} consecutive failures")
            return None

    def is_available(self) -> bool:
        """Check if LLM is available for queries."""
        return self.enabled and self.client is not None and self.failure_count < self.max_failures

    def log_event(self, event_type: str, event_data: Dict[str, Any]) -> None:
        """Log an event for LLM context building.

        Args:
            event_type: Type of event (e.g., "barrel_collected", "navigation_failed")
            event_data: Event details
        """
        event = {
            "time": time.time(),
            "type": event_type,
            "data": event_data
        }
        self.event_history.append(event)

        # Keep history bounded
        if len(self.event_history) > self.max_history:
            self.event_history = self.event_history[-self.max_history:]

    def _get_recent_events_summary(self) -> str:
        """Get a summary of recent events for LLM context."""
        if not self.event_history:
            return "No recent events."

        summary = []
        for event in self.event_history[-5:]:  # Last 5 events
            summary.append(f"- {event['type']}: {json.dumps(event['data'])}")
        return "\n".join(summary)

    def decide_next_action(
        self,
        current_state: str,
        robot_position: Tuple[float, float],
        robot_yaw: float,
        visible_barrels: List[Dict[str, Any]],
        visible_zones: List[Dict[str, Any]],
        has_barrel: bool,
        collected_count: int,
        radiation_level: float,
        sensor_data: Dict[str, float] = None,
        nav_active: bool = False,
        waypoint_idx: int = 0,
    ) -> Optional[str]:
        """Ask LLM to decide the next action based on current situation.

        Args:
            current_state: Current state machine state
            robot_position: (x, y) position in map coordinates
            robot_yaw: Robot orientation in radians
            visible_barrels: List of visible barrels with color, size, position
            visible_zones: List of visible zones with color, size, position
            has_barrel: Whether robot is carrying a barrel
            collected_count: Number of barrels collected so far
            radiation_level: Current radiation level
            sensor_data: LiDAR distances {front, left, right, rear, front_left, front_right}
            nav_active: Whether Nav2 navigation is currently active
            waypoint_idx: Current waypoint index in patrol

        Returns:
            Optional action string, or None to use default behavior
        """
        if not self.is_available():
            return None

        if sensor_data is None:
            sensor_data = {}

        system_prompt = """You are a robot control assistant for a TurtleBot3 barrel collection robot.

TASK: Collect colored barrels (red=contaminated, blue=normal) and deliver them to GREEN zones.
- Red barrels cause radiation when carried - need decontamination at CYAN zone
- Robot has LiDAR (360°) and camera for sensing
- Navigation uses Nav2 stack with pre-built map

SENSOR DATA:
- LiDAR distances in meters (front/left/right/rear/front_left/front_right)
- Camera detects barrels and zones with pixel size (larger = closer)
- Image center x=320, barrel x<320 is left, x>320 is right

STATE MACHINE STATES:
- SEARCHING: Patrolling waypoints looking for barrels
- SCANNING: Rotating at waypoint to scan for barrels
- APPROACHING: Moving toward detected barrel
- RAMMING: Final push to collect barrel (critical, don't interrupt)
- NAVIGATING_TO_GREEN: Delivering barrel to green zone
- NAVIGATING_TO_CYAN: Going to decontaminate
- DELIVERING/DECONTAMINATING: Service calls in progress

YOU ARE IN DIRECT CONTROL MODE - You control the robot's movement directly!

CRITICAL: NEVER use STOP! The robot must ALWAYS be moving or turning to explore!

MOTION COMMANDS:
- FORWARD: Move forward (0.2 m/s) - DEFAULT when path is clear
- FORWARD_SLOW: Move forward slowly (0.1 m/s) - use when close to barrel
- FORWARD_FAST: Move forward faster (0.3 m/s) - use when path is very open
- BACKWARD: Move backward (-0.15 m/s) - only when front blocked
- TURN_LEFT: Rotate left (0.5 rad/s) - avoid right obstacles or scan
- TURN_RIGHT: Rotate right (-0.5 rad/s) - avoid left obstacles or scan
- FORWARD_LEFT: Move forward while turning left - approach barrel on left
- FORWARD_RIGHT: Move forward while turning right - approach barrel on right
- BACKWARD_LEFT: Backup while turning left
- BACKWARD_RIGHT: Backup while turning right

SERVICE COMMANDS:
- PICKUP: Pick up barrel (only when barrel size > 15000 AND front < 0.5m)
- OFFLOAD: Drop barrel (only when green zone visible AND has barrel)

OBSTACLE AVOIDANCE RULES (PRIORITY - check these first!):
1. front < 0.3m → BACKWARD (emergency!)
2. front < 0.5m AND front_left < 0.4m → BACKWARD_RIGHT
3. front < 0.5m AND front_right < 0.4m → BACKWARD_LEFT
4. front_left < 0.35m → TURN_RIGHT or FORWARD_RIGHT
5. front_right < 0.35m → TURN_LEFT or FORWARD_LEFT
6. left < 0.3m → FORWARD_RIGHT (drift away from wall)
7. right < 0.3m → FORWARD_LEFT (drift away from wall)

BARREL COLLECTION LOGIC:
1. If see barrel AND NOT carrying one:
   - x < 250 (far left) → TURN_LEFT
   - x < 320 (left) → FORWARD_LEFT
   - x > 390 (far right) → TURN_RIGHT
   - x > 320 (right) → FORWARD_RIGHT
   - 270 < x < 370 (centered):
     * size < 5000 → FORWARD
     * size 5000-15000 → FORWARD_SLOW
     * size > 15000 AND front < 0.5m → PICKUP

2. If carrying barrel:
   - If see green zone: move toward it (same logic as barrel)
   - If green zone size > 5000 → OFFLOAD
   - If no zone visible → TURN_LEFT to search

3. If no barrel visible AND not carrying:
   - FORWARD if path clear (front > 1.0m)
   - Otherwise TURN_LEFT to scan for barrels

EXPLORATION: When nothing interesting visible, keep moving!
- front > 1.5m → FORWARD_FAST
- front > 0.8m → FORWARD
- front > 0.5m → FORWARD_SLOW
- front < 0.5m → Turn toward more open side (check left vs right distance)

Respond with ONLY ONE command name. NEVER respond with STOP!"""

        # Format barrel info with full details
        barrels_str = "None visible"
        if visible_barrels:
            barrel_details = []
            for i, b in enumerate(visible_barrels[:5]):
                color = b.get('color', 'unknown')
                size = b.get('size', 0)
                x = b.get('x', 320)
                offset = "center" if abs(x - 320) < 50 else ("left" if x < 320 else "right")
                barrel_details.append(f"  [{i}] {color}: size={size:.0f}, x={x:.0f} ({offset})")
            barrels_str = "\n" + "\n".join(barrel_details)

        # Format zone info
        zones_str = "None visible"
        if visible_zones:
            zone_details = []
            for z in visible_zones[:3]:
                color = z.get('color', 'unknown')
                size = z.get('size', 0)
                zone_details.append(f"  {color}: size={size:.0f}")
            zones_str = "\n" + "\n".join(zone_details)

        # Format sensor data
        sensor_str = "No data"
        if sensor_data:
            sensor_str = f"""
  front: {sensor_data.get('front', 10):.2f}m
  front_left: {sensor_data.get('front_left', 10):.2f}m
  front_right: {sensor_data.get('front_right', 10):.2f}m
  left: {sensor_data.get('left', 10):.2f}m
  right: {sensor_data.get('right', 10):.2f}m
  rear: {sensor_data.get('rear', 10):.2f}m"""

        import math
        user_prompt = f"""=== ROBOT STATUS ===
State: {current_state}
Position: ({robot_position[0]:.2f}, {robot_position[1]:.2f}) m
Yaw: {math.degrees(robot_yaw):.1f}°
Nav2 Active: {nav_active}
Waypoint: {waypoint_idx}

=== TASK STATUS ===
Carrying barrel: {has_barrel}
Barrels collected: {collected_count}
Radiation level: {radiation_level:.1f}/100

=== LIDAR (distances in meters) ==={sensor_str}

=== CAMERA (visible objects) ===
Barrels: {barrels_str}
Zones: {zones_str}

=== RECENT EVENTS ===
{self._get_recent_events_summary()}

What action should the robot take?"""

        response = self._query_llm(system_prompt, user_prompt, max_tokens=20)

        if response:
            response = response.upper().strip()
            # Extract first word
            action = response.split()[0] if response.split() else response

            valid_actions = ["CONTINUE", "SEARCH", "APPROACH", "DELIVER", "DECONTAMINATE", "WAIT"]
            if action in valid_actions:
                self._log_info(f"LLM decision: {action}")
                return action

            # Try to match partial
            for valid in valid_actions:
                if valid in response:
                    self._log_info(f"LLM decision (matched): {valid}")
                    return valid

        return None

    def select_target_barrel(
        self,
        visible_barrels: List[Dict[str, Any]],
        robot_position: Tuple[float, float],
    ) -> Optional[int]:
        """Ask LLM to select which barrel to target from visible barrels.

        Args:
            visible_barrels: List of barrels with {color, size, x, y}
            robot_position: Current robot position

        Returns:
            Index of selected barrel, or None to use default
        """
        if not self.is_available() or not visible_barrels:
            return None

        if len(visible_barrels) == 1:
            return 0  # Only one option

        system_prompt = """You are helping a robot select which barrel to collect.
Consider:
- Larger barrels (higher size) are closer and easier to reach
- Centered barrels (x closer to 320) require less turning
- Both red and blue barrels need to be collected

Respond with ONLY the barrel number (0, 1, 2, etc.)."""

        barrels_desc = []
        for i, b in enumerate(visible_barrels[:5]):  # Max 5 barrels
            barrels_desc.append(f"{i}: {b.get('color', 'unknown')} barrel, size={b.get('size', 0):.0f}, x={b.get('x', 320):.0f}")

        user_prompt = f"""Visible barrels:
{chr(10).join(barrels_desc)}

Robot position: ({robot_position[0]:.1f}, {robot_position[1]:.1f})
Image center x = 320 (barrel x < 320 is left, > 320 is right)

Which barrel should the robot target? (respond with just the number)"""

        response = self._query_llm(system_prompt, user_prompt, max_tokens=10)

        if response:
            try:
                idx = int(response.strip())
                if 0 <= idx < len(visible_barrels):
                    self._log_info(f"LLM selected barrel {idx}")
                    return idx
            except ValueError:
                pass

        return None

    def handle_stuck_situation(
        self,
        current_state: str,
        robot_position: Tuple[float, float],
        stuck_duration: float,
        last_actions: List[str],
        sensor_data: Dict[str, float],
    ) -> Optional[str]:
        """Ask LLM for advice when robot is stuck.

        Args:
            current_state: Current state machine state
            robot_position: Current robot position
            stuck_duration: How long robot has been stuck (seconds)
            last_actions: List of recent actions taken
            sensor_data: LiDAR distances (front, left, right, rear)

        Returns:
            Suggested recovery action, or None to use default recovery
        """
        if not self.is_available():
            return None

        system_prompt = """You are helping a stuck robot recover.
Based on sensor data (LiDAR distances), suggest ONE recovery action:
- TURN_LEFT: Rotate left to face open space
- TURN_RIGHT: Rotate right to face open space
- BACKUP: Reverse if there's space behind
- FORWARD: Move forward if there's space ahead
- ESCAPE_LEFT: Turn left and move forward
- ESCAPE_RIGHT: Turn right and move forward

Respond with ONLY the action name."""

        actions_str = ", ".join(last_actions[-5:]) if last_actions else "None"

        user_prompt = f"""Robot is stuck for {stuck_duration:.1f} seconds.
State: {current_state}
Position: ({robot_position[0]:.1f}, {robot_position[1]:.1f})
Recent actions: {actions_str}

LiDAR distances (meters):
- Front: {sensor_data.get('front', 10):.2f}m
- Left: {sensor_data.get('left', 10):.2f}m
- Right: {sensor_data.get('right', 10):.2f}m
- Rear: {sensor_data.get('rear', 10):.2f}m
- Front-left: {sensor_data.get('front_left', 10):.2f}m
- Front-right: {sensor_data.get('front_right', 10):.2f}m

What recovery action should the robot take?"""

        response = self._query_llm(system_prompt, user_prompt, max_tokens=20)

        if response:
            response = response.upper().strip()
            action = response.split()[0] if response.split() else response

            valid_actions = ["TURN_LEFT", "TURN_RIGHT", "BACKUP", "FORWARD", "ESCAPE_LEFT", "ESCAPE_RIGHT"]
            if action in valid_actions:
                self._log_info(f"LLM stuck recovery: {action}")
                return action

        return None

    def analyze_failure(
        self,
        failed_action: str,
        error_message: str,
        context: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        """Ask LLM to analyze a failure and suggest corrections.

        Args:
            failed_action: The action that failed
            error_message: Error message or description
            context: Additional context

        Returns:
            Analysis result with 'cause' and 'suggestion' keys, or None
        """
        if not self.is_available():
            return None

        system_prompt = """You are analyzing robot failures.
Provide a brief analysis with:
1. Likely cause of the failure
2. Suggested correction

Respond in JSON format: {"cause": "...", "suggestion": "..."}"""

        context_str = json.dumps(context, indent=2) if context else "No additional context"

        user_prompt = f"""Failed action: {failed_action}
Error: {error_message}
Context:
{context_str}

Analyze this failure."""

        response = self._query_llm(system_prompt, user_prompt, max_tokens=150)

        if response:
            try:
                # Try to parse as JSON
                result = json.loads(response)
                if 'cause' in result and 'suggestion' in result:
                    self._log_info(f"LLM failure analysis: {result['cause']}")
                    return result
            except json.JSONDecodeError:
                # Try to extract from text
                pass

        return None

    def should_decontaminate(
        self,
        radiation_level: float,
        distance_to_cyan: float,
        has_barrel: bool,
        remaining_time: float,
    ) -> Optional[bool]:
        """Ask LLM whether robot should go decontaminate now.

        Args:
            radiation_level: Current radiation level
            distance_to_cyan: Estimated distance to cyan zone
            has_barrel: Whether robot is carrying a barrel
            remaining_time: Estimated remaining experiment time

        Returns:
            True to decontaminate, False to continue, None for default behavior
        """
        if not self.is_available():
            return None

        # Don't query for obvious cases
        if radiation_level < 20:
            return False  # No need
        if radiation_level > 80:
            return True  # Definitely need

        system_prompt = """You are deciding if a robot should decontaminate.
Consider:
- Higher radiation slows the robot
- Decontamination takes time (travel + process)
- If carrying a barrel, it should be delivered first
- Balance efficiency vs safety

Respond with only YES or NO."""

        user_prompt = f"""Current situation:
- Radiation level: {radiation_level:.1f}/100
- Distance to cyan zone: {distance_to_cyan:.1f}m
- Carrying barrel: {has_barrel}
- Remaining experiment time: {remaining_time:.0f}s

Should the robot go decontaminate now?"""

        response = self._query_llm(system_prompt, user_prompt, max_tokens=10)

        if response:
            response = response.upper().strip()
            if "YES" in response:
                self._log_info("LLM: Decontaminate YES")
                return True
            elif "NO" in response:
                self._log_info("LLM: Decontaminate NO")
                return False

        return None

    # ==================== Vision API 驾驶 ====================

    def vision_drive(
        self,
        image_data: bytes,
        goal: str,
        sensor_data: Dict[str, float],
    ) -> Optional[str]:
        """使用 Vision API 分析图像并决定驾驶动作。

        用于卡住时的智能脱困 - 分析摄像头图像，找到通往目标的路径。

        Args:
            image_data: JPEG 图像数据（bytes）
            goal: 当前目标描述，如 "find red barrel", "reach green zone", "escape to open space"
            sensor_data: LiDAR 距离数据

        Returns:
            驾驶动作: FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, FORWARD_LEFT, FORWARD_RIGHT, 等
        """
        if not self.is_available():
            return None

        # Vision API 使用独立的 rate limit
        now = time.time()
        if now - self.last_vision_call_time < self.min_vision_call_interval:
            return None
        self.last_vision_call_time = now

        try:
            # 将图像编码为 base64
            image_base64 = base64.b64encode(image_data).decode('utf-8')

            # 构建 Vision API 请求
            vision_system_prompt = """You are a robot vision driver. Analyze the camera image and decide the movement command.

GOAL: {goal}

AVAILABLE COMMANDS:
- FORWARD: Move straight ahead (path is clear)
- FORWARD_FAST: Move faster (wide open space ahead)
- FORWARD_LEFT: Move forward while turning left
- FORWARD_RIGHT: Move forward while turning right
- TURN_LEFT: Rotate left in place (to face a better direction)
- TURN_RIGHT: Rotate right in place
- BACKWARD: Reverse (when front is blocked)
- BACKWARD_LEFT: Reverse while turning left
- BACKWARD_RIGHT: Reverse while turning right

DECISION RULES:
1. If you see the target (barrel/zone), move toward it
2. If path is blocked, turn toward open space
3. Prefer moving forward over turning in place
4. Consider the LiDAR data for obstacle distances

OUTPUT: Respond with ONLY ONE command name, nothing else.""".format(goal=goal)

            sensor_str = f"""LiDAR distances:
- Front: {sensor_data.get('front', 10):.2f}m
- Front-left: {sensor_data.get('front_left', 10):.2f}m
- Front-right: {sensor_data.get('front_right', 10):.2f}m
- Left: {sensor_data.get('left', 10):.2f}m
- Right: {sensor_data.get('right', 10):.2f}m
- Rear: {sensor_data.get('rear', 10):.2f}m"""

            response = self.client.chat.completions.create(
                model="gpt-4o-mini",  # Vision 功能需要 4o 系列
                messages=[
                    {"role": "system", "content": vision_system_prompt},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": f"Goal: {goal}\n\n{sensor_str}\n\nAnalyze this camera image and decide the movement:"},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}",
                                    "detail": "low"  # 使用低分辨率减少 token
                                }
                            }
                        ]
                    }
                ],
                temperature=0.2,
                max_tokens=20,
                timeout=8.0  # Vision 需要更长时间
            )

            if response.choices and len(response.choices) > 0:
                self.failure_count = 0
                action = response.choices[0].message.content.strip().upper()
                # 提取第一个词
                action = action.split()[0] if action.split() else action

                valid_actions = [
                    "FORWARD", "FORWARD_FAST", "FORWARD_LEFT", "FORWARD_RIGHT",
                    "TURN_LEFT", "TURN_RIGHT",
                    "BACKWARD", "BACKWARD_LEFT", "BACKWARD_RIGHT"
                ]

                if action in valid_actions:
                    self._log_info(f"[VISION] {goal} -> {action}")
                    return action
                else:
                    self._log_warn(f"[VISION] Invalid action: {action}")
                    return None

        except Exception as e:
            self._log_warn(f"[VISION] Error: {e}")
            self.failure_count += 1
            return None

        return None

    def vision_approach_barrel(
        self,
        image_data: bytes,
        front_dist: float,
        target_color: str = "red",
    ) -> Tuple[Optional[str], bool]:
        """Vision 引导接近桶 - 更激进的 APPROACHING。

        使用 Vision API 分析图像，直接导航到桶旁边。
        当 LiDAR 确认前方有障碍物且足够近时，返回 READY_TO_PICKUP。

        Args:
            image_data: JPEG 图像数据
            front_dist: 前方 LiDAR 距离
            target_color: 目标桶颜色 ("red" 或 "blue")

        Returns:
            Tuple of (action, ready_to_pickup):
            - action: 运动指令 (FORWARD, TURN_LEFT, etc.)
            - ready_to_pickup: 是否可以直接 pickup (前方足够近)
        """
        # 如果前方足够近，可以直接 pickup
        if front_dist < 0.45:
            return ("STOP", True)  # ready to pickup!

        if not self.is_available():
            return (None, False)

        # Vision API 使用独立的 rate limit
        now = time.time()
        if now - self.last_vision_call_time < self.min_vision_call_interval:
            return (None, False)
        self.last_vision_call_time = now

        try:
            image_base64 = base64.b64encode(image_data).decode('utf-8')

            vision_prompt = f"""You are guiding a robot to approach a {target_color} barrel.

TASK: Navigate toward the {target_color} barrel in the image. The barrel is a cylinder.

DISTANCE: Front LiDAR reads {front_dist:.2f}m

COMMANDS:
- FORWARD_FAST: Barrel is centered and far (>1m), move quickly
- FORWARD: Barrel is centered, move toward it
- FORWARD_LEFT: Barrel is on the left, move forward while turning left
- FORWARD_RIGHT: Barrel is on the right, move forward while turning right
- TURN_LEFT: Barrel is far left, need to turn more
- TURN_RIGHT: Barrel is far right, need to turn more
- SEARCH: Cannot see {target_color} barrel, need to search

RULES:
1. If {target_color} barrel is visible and centered -> FORWARD or FORWARD_FAST
2. If {target_color} barrel is on left side -> FORWARD_LEFT or TURN_LEFT
3. If {target_color} barrel is on right side -> FORWARD_RIGHT or TURN_RIGHT
4. If no {target_color} barrel visible -> SEARCH
5. Avoid blue barrels if looking for red!

OUTPUT: Only the command name."""

            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": vision_prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}",
                                    "detail": "low"
                                }
                            }
                        ]
                    }
                ],
                temperature=0.1,
                max_tokens=20,
                timeout=8.0
            )

            if response.choices and len(response.choices) > 0:
                self.failure_count = 0
                action = response.choices[0].message.content.strip().upper()
                action = action.split()[0] if action.split() else action

                valid_actions = [
                    "FORWARD", "FORWARD_FAST", "FORWARD_LEFT", "FORWARD_RIGHT",
                    "TURN_LEFT", "TURN_RIGHT", "SEARCH"
                ]

                if action in valid_actions:
                    self._log_info(f"[VISION-APPROACH] {target_color} barrel -> {action}")
                    return (action, False)
                else:
                    self._log_warn(f"[VISION-APPROACH] Invalid: {action}")
                    return (None, False)

        except Exception as e:
            self._log_warn(f"[VISION-APPROACH] Error: {e}")
            self.failure_count += 1
            return (None, False)

        return (None, False)

    # ==================== LLM 策略决策 ====================

    def get_strategy_decision(
        self,
        has_barrel: bool,
        collected_count: int,
        radiation_level: float,
        visible_barrels: int,
        visible_green_zone: bool,
        visible_cyan_zone: bool,
        time_elapsed: float,
        is_stuck: bool,
    ) -> Optional[str]:
        """LLM 上层策略决策 - 决定机器人当前应该做什么。

        这是高层决策，不是底层运动控制。

        Args:
            has_barrel: 是否正在携带桶
            collected_count: 已收集的桶数量
            radiation_level: 当前辐射等级
            visible_barrels: 可见桶的数量
            visible_green_zone: 是否看到绿色区域
            visible_cyan_zone: 是否看到蓝色洗消区
            time_elapsed: 已过去的时间（秒）
            is_stuck: 是否处于卡住状态

        Returns:
            策略决策: SEARCH_BARREL, COLLECT_BARREL, DELIVER_BARREL, DECONTAMINATE, EXPLORE, ESCAPE
        """
        if not self.is_available():
            return None

        if not self._rate_limit_check():
            return None

        strategy_system_prompt = """You are a high-level strategy planner for a barrel collection robot.

TASK: Collect red barrels and deliver them to green zones. Manage radiation by visiting cyan zones.

AVAILABLE STRATEGIES:
- SEARCH_BARREL: Patrol waypoints to find barrels (when no barrel visible, not carrying)
- COLLECT_BARREL: Go collect a visible barrel (when barrel visible, not carrying)
- DELIVER_BARREL: Deliver carried barrel to green zone (when carrying barrel)
- DECONTAMINATE: Go to cyan zone to reduce radiation (radiation > 60)
- EXPLORE: Random exploration to find new areas
- ESCAPE: Emergency escape from stuck position

DECISION PRIORITY:
1. If stuck -> ESCAPE
2. If radiation > 70 -> DECONTAMINATE (urgent!)
3. If carrying barrel -> DELIVER_BARREL
4. If see barrel and not carrying -> COLLECT_BARREL
5. If radiation > 50 and see cyan -> DECONTAMINATE
6. Otherwise -> SEARCH_BARREL

OUTPUT: Respond with ONLY ONE strategy name."""

        user_prompt = f"""Current situation:
- Carrying barrel: {has_barrel}
- Barrels collected: {collected_count}
- Radiation level: {radiation_level:.0f}/100
- Visible barrels: {visible_barrels}
- See green zone: {visible_green_zone}
- See cyan zone: {visible_cyan_zone}
- Time elapsed: {time_elapsed:.0f}s
- Is stuck: {is_stuck}

What strategy should the robot use?"""

        response = self._query_llm(strategy_system_prompt, user_prompt, max_tokens=20)

        if response:
            strategy = response.upper().strip().split()[0]
            valid_strategies = [
                "SEARCH_BARREL", "COLLECT_BARREL", "DELIVER_BARREL",
                "DECONTAMINATE", "EXPLORE", "ESCAPE"
            ]
            if strategy in valid_strategies:
                self._log_info(f"[STRATEGY] Decision: {strategy}")
                return strategy

        return None


# Global instance (lazy initialization)
_llm_instance: Optional[LLMInterface] = None


def get_llm_interface(logger=None) -> LLMInterface:
    """Get or create the global LLM interface instance."""
    global _llm_instance
    if _llm_instance is None:
        _llm_instance = LLMInterface(logger)
    return _llm_instance


# Convenience functions for backward compatibility
def decide_next_action(
    current_state: str,
    robot_position: Tuple[float, float],
    robot_yaw: float,
    visible_barrels: List[Dict[str, Any]],
    visible_zones: List[Dict[str, Any]],
    has_barrel: bool,
    collected_count: int,
    radiation_level: float,
    sensor_data: Dict[str, float] = None,
    nav_active: bool = False,
    waypoint_idx: int = 0,
) -> Optional[str]:
    """Ask LLM to decide the next action. See LLMInterface.decide_next_action for details."""
    return get_llm_interface().decide_next_action(
        current_state, robot_position, robot_yaw, visible_barrels,
        visible_zones, has_barrel, collected_count, radiation_level,
        sensor_data, nav_active, waypoint_idx
    )


def select_target_barrel(
    visible_barrels: List[Dict[str, Any]],
    robot_position: Tuple[float, float],
) -> Optional[int]:
    """Ask LLM to select target barrel. See LLMInterface.select_target_barrel for details."""
    return get_llm_interface().select_target_barrel(visible_barrels, robot_position)


def handle_stuck_situation(
    current_state: str,
    robot_position: Tuple[float, float],
    stuck_duration: float,
    last_actions: List[str],
    sensor_data: Dict[str, float] = None,
) -> Optional[str]:
    """Ask LLM for stuck recovery. See LLMInterface.handle_stuck_situation for details."""
    if sensor_data is None:
        sensor_data = {}
    return get_llm_interface().handle_stuck_situation(
        current_state, robot_position, stuck_duration, last_actions, sensor_data
    )


def analyze_failure(
    failed_action: str,
    error_message: str,
    context: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """Ask LLM to analyze failure. See LLMInterface.analyze_failure for details."""
    return get_llm_interface().analyze_failure(failed_action, error_message, context)


def should_decontaminate(
    radiation_level: float,
    distance_to_cyan: float,
    has_barrel: bool,
    remaining_time: float,
) -> Optional[bool]:
    """Ask LLM about decontamination. See LLMInterface.should_decontaminate for details."""
    return get_llm_interface().should_decontaminate(
        radiation_level, distance_to_cyan, has_barrel, remaining_time
    )


def log_event(event_type: str, event_data: Dict[str, Any]) -> None:
    """Log an event for LLM context. See LLMInterface.log_event for details."""
    get_llm_interface().log_event(event_type, event_data)
