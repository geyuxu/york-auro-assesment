#!/usr/bin/env python3
"""
Autonomous Cleaner Bot - The Brain of the Robot

This node implements a state machine for autonomous barrel collection:
1. SEARCH: Navigate to waypoints looking for red barrels
2. APPROACH: Visual servoing to approach detected barrel
3. COLLECT: Drive into barrel and call pick_up service
4. DELIVER: Navigate to green zone and offload
5. DECONTAMINATE: If radiation high, go to cyan zone

Uses Nav2 action client for navigation and direct cmd_vel for fine control.
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from assessment_interfaces.msg import BarrelList, ZoneList, Barrel, Zone, RadiationList
from auro_interfaces.srv import ItemRequest
from nav2_msgs.action import NavigateToPose


class State(Enum):
    """Robot state machine states"""
    IDLE = auto()
    SEARCHING = auto()
    APPROACHING_BARREL = auto()
    COLLECTING = auto()
    NAVIGATING_TO_GREEN = auto()
    DELIVERING = auto()
    NAVIGATING_TO_CYAN = auto()
    DECONTAMINATING = auto()


class CleanerBot(Node):
    """
    Autonomous cleaner bot that:
    - Searches for red barrels
    - Collects them using visual servoing
    - Delivers to green zone
    - Decontaminates at cyan zone when needed
    """

    def __init__(self):
        super().__init__('cleaner_bot')

        # Parameters
        self.declare_parameter('use_nav2', True)
        self.declare_parameter('robot_namespace', 'robot1')
        self.use_nav2 = self.get_parameter('use_nav2').value

        # Get robot namespace - prefer parameter over node namespace
        ns = self.get_namespace()
        if ns and ns != '/':
            self.robot_namespace = ns
        else:
            self.robot_namespace = '/' + self.get_parameter('robot_namespace').value

        self.robot_name = self.robot_namespace.strip('/')
        if not self.robot_name:
            self.robot_name = 'robot1'
            self.robot_namespace = '/robot1'

        self.get_logger().info(f'Cleaner Bot starting for {self.robot_name}')

        # Callback groups
        self.callback_group = ReentrantCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # State machine
        self.state = State.IDLE
        self.previous_state = State.IDLE

        # Sensor data
        self.visible_barrels = []
        self.visible_zones = []
        self.current_radiation = 0.0
        self.current_pose = None
        self.current_yaw = 0.0

        # Task tracking
        self.has_barrel = False
        self.collected_count = 0
        self.target_barrel = None
        self.current_waypoint_idx = 0

        # Navigation state (non-blocking)
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_active = False

        # Service call state
        self.service_future = None
        self.service_pending = False

        # Thresholds and constants
        self.RADIATION_THRESHOLD = 50.0
        self.BARREL_SIZE_CLOSE = 100.0   # Barrel size when close enough to collect
        self.BARREL_SIZE_DETECT = 5.0    # Very low: detect barrels as early as possible
        self.VISUAL_CENTER_THRESHOLD = 20  # Tighter alignment
        self.LINEAR_SPEED = 0.18
        self.ANGULAR_SPEED = 0.4

        # Search waypoints - zigzag pattern covering the entire map
        # Map range: X [1.0, 14.0], Y [-19.0, 15.0]
        self.search_waypoints = [
            (1.0, 0.0),
            (14.0, 0.0),
            (14.0, 6.0),
            (1.0, 6.0),
            (1.0, 12.0),
            (14.0, 12.0),
            (14.0, -6.0),
            (1.0, -6.0),
            (1.0, -12.0),
            (14.0, -12.0),
            (14.0, -18.0),
            (1.0, -18.0),
        ]

        # Known zone locations (approximate, will verify with vision)
        self.green_zone_approx = (8.0, 8.0)
        self.cyan_zone_approx = (2.0, 8.0)

        # Publishers - use namespaced topic
        cmd_vel_topic = f'{self.robot_namespace}/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')

        # Subscribers - use RELIABLE QoS to match visual_sensor publisher
        # Use fully qualified topic names with robot namespace
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        barrel_topic = f'{self.robot_namespace}/barrels'
        self.barrel_sub = self.create_subscription(
            BarrelList, barrel_topic, self.barrel_callback, qos_reliable,
            callback_group=self.callback_group)
        self.get_logger().info(f'Subscribing to: {barrel_topic}')

        zone_topic = f'{self.robot_namespace}/zones'
        self.zone_sub = self.create_subscription(
            ZoneList, zone_topic, self.zone_callback, qos_reliable,
            callback_group=self.callback_group)
        self.get_logger().info(f'Subscribing to: {zone_topic}')

        odom_topic = f'{self.robot_namespace}/odom'
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10,
            callback_group=self.callback_group)
        self.get_logger().info(f'Subscribing to: {odom_topic}')

        self.radiation_sub = self.create_subscription(
            RadiationList, '/radiation_levels', self.radiation_callback, 10,
            callback_group=self.callback_group)

        # Service clients
        self.pickup_client = self.create_client(
            ItemRequest, '/pick_up_item',
            callback_group=self.service_callback_group)

        self.offload_client = self.create_client(
            ItemRequest, '/offload_item',
            callback_group=self.service_callback_group)

        self.decontam_client = self.create_client(
            ItemRequest, '/decontaminate',
            callback_group=self.service_callback_group)

        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.pickup_client.wait_for_service(timeout_sec=30.0)
        self.offload_client.wait_for_service(timeout_sec=30.0)
        self.decontam_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Services available!')

        # Nav2 action client (non-blocking approach)
        nav_action_name = f'{self.robot_namespace}/navigate_to_pose'
        self.get_logger().info(f'Creating Nav2 action client: {nav_action_name}')
        self.nav_client = ActionClient(
            self, NavigateToPose, nav_action_name,
            callback_group=self.callback_group)

        # Wait for Nav2 action server
        if self.use_nav2:
            self.get_logger().info('Waiting for Nav2 action server...')
            if not self.nav_client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error('Nav2 action server not available!')
                self.use_nav2 = False
            else:
                self.get_logger().info('Nav2 action server is ready!')

        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.callback_group)

        # Startup delay to let everything initialize
        self.startup_complete = False
        self.startup_timer = self.create_timer(
            3.0, self._startup_complete, callback_group=self.callback_group)

        self.get_logger().info(f'Cleaner Bot initialized for {self.robot_name}')

    def _startup_complete(self):
        """Called after startup delay"""
        self.startup_timer.cancel()
        self.startup_complete = True
        self.state = State.SEARCHING
        self.get_logger().info('Startup complete, beginning autonomous operation!')

    # ==================== CALLBACKS ====================

    def barrel_callback(self, msg: BarrelList):
        """Process detected barrels from visual sensor"""
        self.visible_barrels = msg.data
        # Debug: log when barrels are detected
        if msg.data:
            for b in msg.data:
                self.get_logger().debug(
                    f'Barrel detected: colour={b.colour}, x={b.x}, y={b.y}, size={b.size}')

    def zone_callback(self, msg: ZoneList):
        """Process detected zones from visual sensor"""
        self.visible_zones = msg.data

    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def radiation_callback(self, msg: RadiationList):
        """Update radiation level for this robot"""
        for rad in msg.data:
            if rad.robot_id == self.robot_name:
                self.current_radiation = rad.level
                break

    # ==================== HELPER FUNCTIONS ====================

    def get_red_barrels(self):
        """Filter for red barrels only"""
        return [b for b in self.visible_barrels if b.colour == Barrel.RED]

    def get_green_zones(self):
        """Filter for green zones"""
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_GREEN]

    def get_cyan_zones(self):
        """Filter for cyan zones"""
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_CYAN]

    def stop_robot(self):
        """Stop all robot motion"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def move_forward(self, speed=None):
        """Move robot forward"""
        if speed is None:
            speed = self.LINEAR_SPEED
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

    def rotate(self, angular_speed=None):
        """Rotate robot"""
        if angular_speed is None:
            angular_speed = self.ANGULAR_SPEED
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

    def move_towards_target(self, x_offset, size):
        """
        Visual servoing: move towards a target based on camera x offset and size.
        """
        twist = Twist()

        # Rotate to center the target
        if abs(x_offset) > self.VISUAL_CENTER_THRESHOLD:
            twist.angular.z = -0.002 * x_offset
            twist.angular.z = max(-self.ANGULAR_SPEED, min(self.ANGULAR_SPEED, twist.angular.z))
        else:
            twist.angular.z = 0.0

        # Move forward if not too close
        if size < self.BARREL_SIZE_CLOSE:
            twist.linear.x = self.LINEAR_SPEED * 0.8
        else:
            twist.linear.x = self.LINEAR_SPEED * 0.3

        self.cmd_vel_pub.publish(twist)

    def send_nav_goal(self, x, y, yaw=0.0):
        """Send a navigation goal using Nav2 action client (non-blocking)"""
        if not self.use_nav2:
            self.get_logger().warn('Nav2 not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending nav goal to ({x:.2f}, {y:.2f})')

        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._nav_goal_response_callback)
        self.nav_active = True
        return True

    def _nav_goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected!')
            self.nav_active = False
            self.nav_goal_handle = None
            return

        self.get_logger().info('Navigation goal accepted!')
        self.nav_goal_handle = goal_handle

        # Get result asynchronously
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        """Callback when navigation completes"""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation was cancelled')
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')

        self.nav_active = False
        self.nav_goal_handle = None
        self.nav_result_future = None

    def cancel_navigation(self):
        """Cancel current navigation goal"""
        if self.nav_goal_handle is not None:
            self.get_logger().info('Cancelling navigation...')
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            self.nav_active = False
            self.nav_goal_handle = None

    def is_navigation_active(self):
        """Check if navigation is currently active"""
        return self.nav_active

    # ==================== STATE MACHINE ====================

    def control_loop(self):
        """Main control loop - state machine"""
        if not self.startup_complete:
            return

        # Priority check: High radiation -> go decontaminate
        if (self.current_radiation > self.RADIATION_THRESHOLD and
            self.state not in [State.NAVIGATING_TO_CYAN, State.DECONTAMINATING]):
            self.get_logger().warn(
                f'Radiation level {self.current_radiation:.1f} exceeds threshold! Going to decontaminate.')
            self.cancel_navigation()
            self.previous_state = self.state
            self.state = State.NAVIGATING_TO_CYAN

        # State machine
        if self.state == State.IDLE:
            self.stop_robot()

        elif self.state == State.SEARCHING:
            self._handle_searching()

        elif self.state == State.APPROACHING_BARREL:
            self._handle_approaching()

        elif self.state == State.COLLECTING:
            self._handle_collecting()

        elif self.state == State.NAVIGATING_TO_GREEN:
            self._handle_navigating_to_green()

        elif self.state == State.DELIVERING:
            self._handle_delivering()

        elif self.state == State.NAVIGATING_TO_CYAN:
            self._handle_navigating_to_cyan()

        elif self.state == State.DECONTAMINATING:
            self._handle_decontaminating()

    def _handle_searching(self):
        """Search state: Navigate to waypoints (pure patrol for now)"""
        # TODO: 目标二将添加桶检测逻辑
        # 当前只做纯巡逻

        # Continue patrol via waypoints
        if not self.is_navigation_active():
            waypoint = self.search_waypoints[self.current_waypoint_idx]
            self.get_logger().info(
                f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.search_waypoints)}: ({waypoint[0]}, {waypoint[1]})')
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
            self.send_nav_goal(waypoint[0], waypoint[1])

    def _handle_approaching(self):
        """Approach state: Visual servoing towards detected barrel"""
        red_barrels = self.get_red_barrels()

        if not red_barrels:
            self.get_logger().warn('Lost sight of barrel during approach!')
            self.target_barrel = None
            self.state = State.SEARCHING
            return

        largest = max(red_barrels, key=lambda b: b.size)
        self.target_barrel = largest

        # Check if close enough to collect
        if largest.size >= self.BARREL_SIZE_CLOSE:
            self.get_logger().info('Close enough to barrel! Attempting collection...')
            self.stop_robot()
            self.state = State.COLLECTING
            self.service_pending = False
            return

        # Visual servoing towards barrel
        self.move_towards_target(largest.x, largest.size)

    def _handle_collecting(self):
        """Collect state: Drive into barrel and call pickup service"""
        # TODO: 目标二将实现此功能
        # 当前只是占位，直接返回搜索状态
        self.get_logger().info('COLLECTING state - not implemented yet, returning to search')
        self.state = State.SEARCHING

    def _handle_navigating_to_green(self):
        """Navigate to green zone for delivery"""
        if not self.is_navigation_active():
            self.get_logger().info('Navigating to GREEN ZONE for delivery...')
            self.send_nav_goal(self.green_zone_approx[0], self.green_zone_approx[1])
        else:
            # Check if we can see green zone while navigating
            green_zones = self.get_green_zones()
            if green_zones and green_zones[0].size > 100:
                self.get_logger().info('Green zone visible and close!')
                self.cancel_navigation()
                self.state = State.DELIVERING
                self.service_pending = False

    def _handle_delivering(self):
        """Deliver state: Offload barrel at green zone"""
        green_zones = self.get_green_zones()

        if green_zones:
            zone = green_zones[0]
            if abs(zone.x) > self.VISUAL_CENTER_THRESHOLD:
                self.rotate(-0.002 * zone.x)
                return

        # Check if service call already pending
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                try:
                    result = self.service_future.result()
                    if result.success:
                        self.get_logger().info(f'DELIVERED BARREL! Message: {result.message}')
                        self.get_logger().info(f'Total collected: {self.collected_count}')
                        self.has_barrel = False
                        self.stop_robot()
                        self.state = State.SEARCHING
                    else:
                        self.get_logger().warn(f'Delivery failed: {result.message}')
                        self.move_forward(self.LINEAR_SPEED * 0.3)
                except Exception as e:
                    self.get_logger().error(f'Service call error: {e}')
                    self.state = State.SEARCHING
                self.service_pending = False
                self.service_future = None
            return

        # Start service call
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        self.service_future = self.offload_client.call_async(request)
        self.service_pending = True
        self.get_logger().info(f'Calling offload_item for {self.robot_name}...')

    def _handle_navigating_to_cyan(self):
        """Navigate to cyan zone for decontamination"""
        if not self.is_navigation_active():
            self.get_logger().info('Navigating to CYAN ZONE for decontamination...')
            self.send_nav_goal(self.cyan_zone_approx[0], self.cyan_zone_approx[1])
        else:
            cyan_zones = self.get_cyan_zones()
            if cyan_zones and cyan_zones[0].size > 100:
                self.get_logger().info('Cyan zone visible and close!')
                self.cancel_navigation()
                self.state = State.DECONTAMINATING
                self.service_pending = False

    def _handle_decontaminating(self):
        """Decontaminate at cyan zone"""
        cyan_zones = self.get_cyan_zones()

        if cyan_zones:
            zone = cyan_zones[0]
            if abs(zone.x) > self.VISUAL_CENTER_THRESHOLD:
                self.rotate(-0.002 * zone.x)
                return

        # Check if service call already pending
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                try:
                    result = self.service_future.result()
                    if result.success:
                        self.get_logger().info(f'DECONTAMINATED! Message: {result.message}')
                        self.stop_robot()
                        if self.has_barrel:
                            self.state = State.NAVIGATING_TO_GREEN
                        else:
                            self.state = State.SEARCHING
                    else:
                        self.get_logger().warn(f'Decontamination failed: {result.message}')
                        self.move_forward(self.LINEAR_SPEED * 0.3)
                except Exception as e:
                    self.get_logger().error(f'Service call error: {e}')
                    self.state = State.SEARCHING
                self.service_pending = False
                self.service_future = None
            return

        # Start service call
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        self.service_future = self.decontam_client.call_async(request)
        self.service_pending = True
        self.get_logger().info(f'Calling decontaminate for {self.robot_name}...')


def main(args=None):
    rclpy.init(args=args)

    node = CleanerBot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    except ExternalShutdownException:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
