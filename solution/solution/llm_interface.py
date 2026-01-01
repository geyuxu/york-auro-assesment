"""
LLM Interface for Cleaner Bot

This module provides an interface for LLM-based decision making.
Uses OpenAI API (or compatible API) for intelligent decision support.
"""

import os
import json
import time
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
        self.model = os.getenv('OPENAI_MODEL', 'gpt-4o-mini')

        # Rate limiting
        self.last_call_time = 0
        self.min_call_interval = 2.0  # Minimum 2 seconds between calls

        # Failure tracking
        self.failure_count = 0
        self.max_failures = 5  # Disable LLM after 5 consecutive failures

        # Event history for context
        self.event_history: List[Dict[str, Any]] = []
        self.max_history = 20

        # Initialize client
        if self.enabled and self.api_key:
            try:
                self.client = OpenAI(api_key=self.api_key, base_url=self.api_base)
                self._log_info(f"LLM interface initialized (model: {self.model})")
            except Exception as e:
                self._log_warn(f"Failed to initialize OpenAI client: {e}")
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

ACTIONS - You can either use high-level state commands OR direct motion control:

HIGH-LEVEL COMMANDS:
- CONTINUE: Let default state machine handle it
- SEARCH: Force return to searching state
- APPROACH: Approach visible barrel (only if barrels visible)
- DELIVER: Go to green zone (only if carrying barrel)
- DECONTAMINATE: Go to cyan zone (if radiation high)

DIRECT MOTION CONTROL (use when you need precise control):
- FORWARD: Move forward (0.2 m/s)
- FORWARD_SLOW: Move forward slowly (0.1 m/s)
- BACKWARD: Move backward (-0.15 m/s)
- TURN_LEFT: Rotate left (0.5 rad/s)
- TURN_RIGHT: Rotate right (-0.5 rad/s)
- TURN_LEFT_SLOW: Rotate left slowly (0.3 rad/s)
- TURN_RIGHT_SLOW: Rotate right slowly (-0.3 rad/s)
- STOP: Stop all motion

COMBINED MOTION (move and turn simultaneously):
- FORWARD_LEFT: Move forward while turning left
- FORWARD_RIGHT: Move forward while turning right
- BACKWARD_LEFT: Move backward while turning left
- BACKWARD_RIGHT: Move backward while turning right

DECISION GUIDELINES:
1. If barrel visible and NOT carrying one:
   - If barrel is centered (x near 320) and large -> FORWARD to approach
   - If barrel is on left (x < 270) -> TURN_LEFT or FORWARD_LEFT to center it
   - If barrel is on right (x > 370) -> TURN_RIGHT or FORWARD_RIGHT to center it
2. If carrying barrel -> CONTINUE (let state machine deliver)
3. If front distance < 0.3m and need to move forward -> BACKWARD first, then turn
4. If stuck (not moving for a while) -> try BACKWARD then TURN_LEFT or TURN_RIGHT
5. If see barrel very close (size > 10000) -> FORWARD_SLOW to not overshoot

IMPORTANT:
- Use direct motion when you see a barrel and want to approach it precisely
- Barrel x < 320 means it's on the LEFT side of camera
- Barrel x > 320 means it's on the RIGHT side of camera
- Larger barrel size = closer to robot

Respond with ONLY the action name."""

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
