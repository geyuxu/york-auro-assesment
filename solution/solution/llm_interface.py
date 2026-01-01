"""
LLM Interface for Cleaner Bot

This module provides an interface for LLM-based decision making.
All functions return None or default values until implemented.
"""

from typing import Optional, List, Tuple, Dict, Any


def decide_next_action(
    current_state: str,
    robot_position: Tuple[float, float],
    robot_yaw: float,
    visible_barrels: List[Dict[str, Any]],
    visible_zones: List[Dict[str, Any]],
    has_barrel: bool,
    collected_count: int,
    radiation_level: float,
) -> Optional[str]:
    """
    Ask LLM to decide the next action based on current situation.

    Args:
        current_state: Current state machine state (e.g., "SEARCHING", "APPROACHING")
        robot_position: (x, y) position in map coordinates
        robot_yaw: Robot orientation in radians
        visible_barrels: List of visible barrels with color, size, position
        visible_zones: List of visible zones with color, size, position
        has_barrel: Whether robot is carrying a barrel
        collected_count: Number of barrels collected so far
        radiation_level: Current radiation level (0.0 - 1.0)

    Returns:
        Optional action string, or None to use default behavior
        Possible actions: "SEARCH", "APPROACH", "PICKUP", "DELIVER", "DECONTAMINATE", "WAIT"
    """
    # TODO: Implement LLM call
    return None


def select_target_barrel(
    visible_barrels: List[Dict[str, Any]],
    robot_position: Tuple[float, float],
) -> Optional[int]:
    """
    Ask LLM to select which barrel to target from visible barrels.

    Args:
        visible_barrels: List of barrels with {color, size, x, y}
        robot_position: Current robot position

    Returns:
        Index of selected barrel, or None to use default (largest red barrel)
    """
    # TODO: Implement LLM call
    return None


def plan_search_path(
    current_position: Tuple[float, float],
    explored_areas: List[Tuple[float, float]],
    map_bounds: Tuple[float, float, float, float],
) -> Optional[List[Tuple[float, float]]]:
    """
    Ask LLM to plan a search path for finding barrels.

    Args:
        current_position: Current robot position
        explored_areas: List of already explored positions
        map_bounds: (min_x, min_y, max_x, max_y) of the map

    Returns:
        List of waypoints to visit, or None to use default waypoints
    """
    # TODO: Implement LLM call
    return None


def handle_stuck_situation(
    current_state: str,
    robot_position: Tuple[float, float],
    stuck_duration: float,
    last_actions: List[str],
) -> Optional[str]:
    """
    Ask LLM for advice when robot is stuck.

    Args:
        current_state: Current state machine state
        robot_position: Current robot position
        stuck_duration: How long robot has been stuck (seconds)
        last_actions: List of recent actions taken

    Returns:
        Suggested recovery action, or None to use default recovery
    """
    # TODO: Implement LLM call
    return None


def analyze_failure(
    failed_action: str,
    error_message: str,
    context: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """
    Ask LLM to analyze a failure and suggest corrections.

    Args:
        failed_action: The action that failed
        error_message: Error message or description
        context: Additional context (state, position, sensor data, etc.)

    Returns:
        Analysis result with 'cause' and 'suggestion' keys, or None
    """
    # TODO: Implement LLM call
    return None


def should_decontaminate(
    radiation_level: float,
    distance_to_cyan: float,
    has_barrel: bool,
    remaining_time: float,
) -> Optional[bool]:
    """
    Ask LLM whether robot should go decontaminate now.

    Args:
        radiation_level: Current radiation level (0.0 - 1.0)
        distance_to_cyan: Estimated distance to cyan zone
        has_barrel: Whether robot is carrying a barrel
        remaining_time: Estimated remaining experiment time

    Returns:
        True to decontaminate, False to continue, None for default behavior
    """
    # TODO: Implement LLM call
    return None


def log_event(
    event_type: str,
    event_data: Dict[str, Any],
) -> None:
    """
    Log an event for LLM context building (non-blocking).

    Args:
        event_type: Type of event (e.g., "barrel_collected", "navigation_failed")
        event_data: Event details
    """
    # TODO: Implement event logging for LLM context
    pass
