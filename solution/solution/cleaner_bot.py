#!/usr/bin/env python3
"""
Autonomous Cleaner Bot - SIMPLIFIED VERSION

Strategy: Use Nav2 for ALL navigation, including approaching barrels.
Only use direct cmd_vel for the final "ramming" phase when very close.

State Machine:
1. SEARCHING: Nav2 patrol waypoints, watch for big red blobs
2. APPROACHING: Nav2 to barrel location (estimated from camera)
3. RAMMING: Direct forward motion to drive through barrel
4. NAVIGATING_TO_GREEN: Nav2 to delivery zone
5. DELIVERING: Call offload service
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSPresetProfiles
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import BarrelList, ZoneList, Barrel, Zone, RadiationList
from auro_interfaces.srv import ItemRequest
from nav2_msgs.action import NavigateToPose


class State(Enum):
    IDLE = auto()
    SEARCHING = auto()
    APPROACHING = auto()  # Nav2 getting close to barrel
    RAMMING = auto()      # Direct forward to push through barrel
    NAVIGATING_TO_GREEN = auto()
    DELIVERING = auto()
    NAVIGATING_TO_CYAN = auto()
    DECONTAMINATING = auto()


class CleanerBot(Node):

    def __init__(self):
        super().__init__('cleaner_bot')

        # Parameters
        self.declare_parameter('use_nav2', True)
        self.declare_parameter('robot_namespace', 'robot1')
        self.use_nav2 = self.get_parameter('use_nav2').value

        # Get robot namespace
        ns = self.get_namespace()
        if ns and ns != '/':
            self.robot_namespace = ns
        else:
            self.robot_namespace = '/' + self.get_parameter('robot_namespace').value

        self.robot_name = self.robot_namespace.strip('/')
        if not self.robot_name:
            self.robot_name = 'robot1'
            self.robot_namespace = '/robot1'

        self.get_logger().info(f'Cleaner Bot (SIMPLE) starting for {self.robot_name}')

        # Callback groups
        self.callback_group = ReentrantCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # State machine
        self.state = State.IDLE

        # Sensor data
        self.visible_barrels = []
        self.visible_zones = []
        self.current_radiation = 0.0
        self.current_pose = None
        self.current_yaw = 0.0
        self.front_min_dist = 10.0
        self.left_min_dist = 10.0
        self.right_min_dist = 10.0

        # Task tracking
        self.has_barrel = False
        self.collected_count = 0
        self.current_waypoint_idx = 0

        # Navigation state
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_active = False

        # Service call state
        self.service_future = None
        self.service_pending = False

        # Simple thresholds
        self.RADIATION_THRESHOLD = 50.0
        self.BARREL_SIZE_START = 500      # Start approaching when barrel this big
        self.BARREL_SIZE_COMMIT = 3000    # Switch to ramming when barrel this big
        self.IMAGE_CENTER_X = 320
        self.LINEAR_SPEED = 0.2
        self.ANGULAR_SPEED = 0.5
        self.SCAN_THRESHOLD = 0.35
        self.SCAN_STOP_THRESHOLD = 0.25

        # Approaching state
        self.approach_counter = 0
        self.MAX_APPROACH_ATTEMPTS = 100  # 10 seconds at 10Hz
        self.lost_sight_counter = 0

        # Ramming state
        self.ram_counter = 0
        self.MAX_RAM_ATTEMPTS = 50  # 5 seconds at 10Hz

        # Search waypoints - cover the map
        self.search_waypoints = [
            (2, 3),
            (9.8, 6.0),
            (10.0, 6.0),
            (9.8, 9.0),
            (9.8, 12.0),
            (9.8, 15.0),
            (7, 15.0),
            (7, 12.0),
            (7, 10.0),
            (7, 9.0),
            (14, 9.0),
            (14, 15.0),
        ]

        # Zone locations
        self.green_zone_approx = (8.0, 8.0)
        self.cyan_zone_approx = (2.0, 8.0)

        # Publishers
        cmd_vel_topic = f'{self.robot_namespace}/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Subscribers
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        barrel_topic = f'{self.robot_namespace}/barrels'
        self.barrel_sub = self.create_subscription(
            BarrelList, barrel_topic, self.barrel_callback, qos_reliable,
            callback_group=self.callback_group)

        zone_topic = f'{self.robot_namespace}/zones'
        self.zone_sub = self.create_subscription(
            ZoneList, zone_topic, self.zone_callback, qos_reliable,
            callback_group=self.callback_group)

        odom_topic = f'{self.robot_namespace}/odom'
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10,
            callback_group=self.callback_group)

        self.radiation_sub = self.create_subscription(
            RadiationList, '/radiation_levels', self.radiation_callback, 10,
            callback_group=self.callback_group)

        scan_topic = f'{self.robot_namespace}/scan'
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
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
        self.get_logger().info('Services ready!')

        # Nav2 action client
        nav_action_name = f'{self.robot_namespace}/navigate_to_pose'
        self.nav_client = ActionClient(
            self, NavigateToPose, nav_action_name,
            callback_group=self.callback_group)

        if self.use_nav2:
            self.get_logger().info('Waiting for Nav2...')
            if not self.nav_client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error('Nav2 not available!')
                self.use_nav2 = False
            else:
                self.get_logger().info('Nav2 ready!')

        # Control timer
        self.control_timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.callback_group)

        # Startup delay
        self.startup_complete = False
        self.startup_timer = self.create_timer(
            3.0, self._startup_complete, callback_group=self.callback_group)

    def _startup_complete(self):
        self.startup_timer.cancel()
        self.startup_complete = True
        self.state = State.SEARCHING
        self.get_logger().info('Starting autonomous operation!')

    # ==================== CALLBACKS ====================

    def barrel_callback(self, msg):
        self.visible_barrels = msg.data

    def zone_callback(self, msg):
        self.visible_zones = msg.data

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def radiation_callback(self, msg):
        for rad in msg.data:
            if rad.robot_id == self.robot_name:
                self.current_radiation = rad.level
                break

    def scan_callback(self, msg):
        if len(msg.ranges) < 360:
            return
        # Front: -20 to +20 degrees
        front_ranges = list(msg.ranges[340:360]) + list(msg.ranges[0:20])
        valid_front = [r for r in front_ranges if 0.01 < r < 10.0]
        self.front_min_dist = min(valid_front) if valid_front else 10.0

        # Left: 20 to 70 degrees
        left_ranges = list(msg.ranges[20:70])
        valid_left = [r for r in left_ranges if 0.01 < r < 10.0]
        self.left_min_dist = min(valid_left) if valid_left else 10.0

        # Right: 290 to 340 degrees
        right_ranges = list(msg.ranges[290:340])
        valid_right = [r for r in right_ranges if 0.01 < r < 10.0]
        self.right_min_dist = min(valid_right) if valid_right else 10.0

    # ==================== HELPERS ====================

    def get_red_barrels(self):
        return [b for b in self.visible_barrels if b.colour == Barrel.RED]

    def get_green_zones(self):
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_GREEN]

    def get_cyan_zones(self):
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_CYAN]

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def send_nav_goal(self, x, y, yaw=0.0):
        if not self.use_nav2:
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Nav goal: ({x:.1f}, {y:.1f})')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._nav_goal_response)
        self.nav_active = True
        return True

    def _nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav goal rejected!')
            self.nav_active = False
            return
        self.nav_goal_handle = goal_handle
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav succeeded!')
            # Only advance waypoint on successful arrival
            if self.state == State.SEARCHING and not self.has_barrel:
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
        self.nav_active = False
        self.nav_goal_handle = None

    def cancel_navigation(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_active = False
            self.nav_goal_handle = None

    def is_navigation_active(self):
        return self.nav_active

    # ==================== STATE MACHINE ====================

    def control_loop(self):
        if not self.startup_complete:
            return

        # Priority: Radiation check
        if (self.current_radiation > self.RADIATION_THRESHOLD and
            self.state not in [State.NAVIGATING_TO_CYAN, State.DECONTAMINATING]):
            self.get_logger().warn(f'High radiation! {self.current_radiation:.1f}')
            self.cancel_navigation()
            self.state = State.NAVIGATING_TO_CYAN

        # State machine
        if self.state == State.IDLE:
            self.stop_robot()

        elif self.state == State.SEARCHING:
            self._handle_searching()

        elif self.state == State.APPROACHING:
            self._handle_approaching()

        elif self.state == State.RAMMING:
            self._handle_ramming()

        elif self.state == State.NAVIGATING_TO_GREEN:
            self._handle_nav_to_green()

        elif self.state == State.DELIVERING:
            self._handle_delivering()

        elif self.state == State.NAVIGATING_TO_CYAN:
            self._handle_nav_to_cyan()

        elif self.state == State.DECONTAMINATING:
            self._handle_decontaminating()

    def _handle_searching(self):
        """Search: Use Nav2 to patrol, watch for barrels."""
        if self.has_barrel:
            return

        red_barrels = self.get_red_barrels()

        if red_barrels:
            # Get largest barrel
            best = max(red_barrels, key=lambda b: b.size)

            # Start visual approach when barrel is visible and big enough
            if best.size > self.BARREL_SIZE_START:
                x_offset = best.x - self.IMAGE_CENTER_X

                # SAFETY: Check if we're stuck (surrounded by walls)
                if self.front_min_dist < 0.3 and self.left_min_dist < 0.3 and self.right_min_dist < 0.3:
                    self.get_logger().warn('STUCK! Walls on all sides - let Nav2 handle escape')
                    self.stop_robot()
                    return  # Let Nav2 find a way out

                # If barrel is VERY big but not centered, we're probably stuck next to it
                # Skip alignment and go straight to ramming if barrel is huge
                if best.size > self.BARREL_SIZE_COMMIT:
                    self.get_logger().info(f'BARREL HUGE! size={best.size:.0f} -> RAMMING directly')
                    self.cancel_navigation()
                    self.ram_counter = 0
                    self.state = State.RAMMING
                    return

                # If barrel not centered, turn toward it first
                if abs(x_offset) > 80:  # More than ~80 pixels off center
                    self.cancel_navigation()

                    # Check for wall corners before turning!
                    want_turn_left = x_offset < 0  # barrel on left
                    want_turn_right = x_offset > 0  # barrel on right

                    if want_turn_left and self.left_min_dist < 0.35:
                        self.get_logger().warn(f'Want to turn left but wall corner at {self.left_min_dist:.2f}m - abort')
                        self.stop_robot()
                        return  # Stay in SEARCHING, Nav2 will continue

                    if want_turn_right and self.right_min_dist < 0.35:
                        self.get_logger().warn(f'Want to turn right but wall corner at {self.right_min_dist:.2f}m - abort')
                        self.stop_robot()
                        return  # Stay in SEARCHING, Nav2 will continue

                    # Also check front - if front is blocked, we can't approach anyway
                    if self.front_min_dist < 0.35:
                        self.get_logger().warn(f'Front blocked at {self.front_min_dist:.2f}m - abort')
                        self.stop_robot()
                        return

                    twist = Twist()
                    twist.angular.z = -0.006 * x_offset  # Turn toward barrel
                    twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info(f'BARREL SEEN! x={best.x:.0f} size={best.size:.0f} - ALIGNING')
                    return

                # Barrel is centered enough, start approaching
                self.get_logger().info(f'BARREL ALIGNED! x={best.x:.0f} size={best.size:.0f} -> APPROACHING')
                self.cancel_navigation()
                self.approach_counter = 0
                self.lost_sight_counter = 0
                self.state = State.APPROACHING
                return

        # Continue patrol
        if not self.is_navigation_active():
            wp = self.search_waypoints[self.current_waypoint_idx]
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1}/{len(self.search_waypoints)}: ({wp[0]}, {wp[1]})')
            self.send_nav_goal(wp[0], wp[1])

    def _handle_approaching(self):
        """Visual servoing approach to barrel with obstacle avoidance.

        KEY SAFETY RULE: Only move forward if path is clear.
        If there's an obstacle but barrel is small = wall blocking, don't move forward.
        """
        self.approach_counter += 1

        # Timeout check
        if self.approach_counter > self.MAX_APPROACH_ATTEMPTS:
            self.get_logger().warn('Approach timeout, back to search')
            self.stop_robot()
            self.state = State.SEARCHING
            return

        red_barrels = self.get_red_barrels()

        if not red_barrels:
            self.lost_sight_counter += 1
            if self.lost_sight_counter > 30:  # 3 seconds
                self.get_logger().warn('Lost barrel, back to search')
                self.stop_robot()
                self.state = State.SEARCHING
                return
            # Stop and wait, don't blindly move
            self.stop_robot()
            return

        # Found barrel
        self.lost_sight_counter = 0
        best = max(red_barrels, key=lambda b: b.size)

        # Check if close enough to start ramming
        if best.size > self.BARREL_SIZE_COMMIT:
            self.get_logger().info(f'BARREL CLOSE! size={best.size:.0f} -> RAMMING')
            self.ram_counter = 0
            self.state = State.RAMMING
            return

        # Calculate steering - keep barrel centered
        x_offset = best.x - self.IMAGE_CENTER_X
        # Use stronger gain for steering to keep barrel centered
        angular_z = -0.006 * x_offset

        # SAFETY CHECK: Is there something blocking our path?
        # If front distance is small but barrel is not big, it's a WALL
        if self.front_min_dist < 0.4:
            # Is it the barrel or a wall?
            # Barrel at 0.4m should have size roughly > 1500
            # If size is small, it's seeing barrel through wall/door
            if best.size < 1500:
                self.get_logger().warn(
                    f'WALL DETECTED! front={self.front_min_dist:.2f}m but barrel small={best.size:.0f}. '
                    f'Aborting approach.'
                )
                self.stop_robot()
                # Give up on this barrel, go back to search
                self.state = State.SEARCHING
                return

        # Determine if we can move forward
        linear_x = 0.0

        # Only move forward if path is reasonably clear
        if self.front_min_dist > 0.5:
            linear_x = self.LINEAR_SPEED
        elif self.front_min_dist > 0.35:
            # Close but maybe it's the barrel - check size
            if best.size > 1000:
                linear_x = 0.1  # Slow approach
            else:
                linear_x = 0.0  # Don't move, something's in the way
        else:
            # Very close - only proceed if barrel is big
            if best.size > 2000:
                linear_x = 0.1
                self.get_logger().info('Close to barrel, slow approach')
            else:
                linear_x = 0.0

        # Side obstacle avoidance (wall corners)
        # If side obstacle is very close, we need to stop and reconsider
        if self.left_min_dist < 0.25 or self.right_min_dist < 0.25:
            self.get_logger().warn(
                f'Wall corner too close! L={self.left_min_dist:.2f} R={self.right_min_dist:.2f} - abort approach'
            )
            self.stop_robot()
            self.state = State.SEARCHING
            return

        # If side obstacle moderately close, steer away
        if self.left_min_dist < 0.4:
            angular_z -= 0.3  # Turn right more aggressively
        if self.right_min_dist < 0.4:
            angular_z += 0.3  # Turn left more aggressively

        # Clamp angular velocity
        angular_z = max(-0.4, min(0.4, angular_z))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

        # Log occasionally
        if self.approach_counter % 10 == 0:
            self.get_logger().info(
                f'APPROACH: size={best.size:.0f} x={best.x:.0f} '
                f'front={self.front_min_dist:.2f} L={self.left_min_dist:.2f} R={self.right_min_dist:.2f} '
                f'lin={linear_x:.2f}'
            )

    def _handle_ramming(self):
        """Ram: Drive forward to push through barrel, try pickup."""
        self.ram_counter += 1

        # Timeout
        if self.ram_counter > self.MAX_RAM_ATTEMPTS:
            self.get_logger().warn('Ram timeout, back to search')
            self.stop_robot()
            self.state = State.SEARCHING
            return

        # SAFETY: If we're stuck (can't move forward), abort early
        if self.front_min_dist < 0.15 and self.left_min_dist < 0.25 and self.right_min_dist < 0.25:
            self.get_logger().warn(f'STUCK during ram! front={self.front_min_dist:.2f} - abort')
            self.stop_robot()
            self.state = State.SEARCHING
            return

        # Drive forward
        twist = Twist()
        twist.linear.x = self.LINEAR_SPEED

        # Steering to keep barrel centered during ramming
        red_barrels = self.get_red_barrels()
        if red_barrels:
            best = max(red_barrels, key=lambda b: b.size)
            x_offset = best.x - self.IMAGE_CENTER_X
            twist.angular.z = -0.005 * x_offset  # Stronger steering during ram
            twist.angular.z = max(-0.4, min(0.4, twist.angular.z))

        self.cmd_vel_pub.publish(twist)

        # Try pickup every cycle
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        future = self.pickup_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)

        if future.done():
            result = future.result()
            if result and result.success:
                self.get_logger().info(f'COLLECTED! {result.message}')
                self.has_barrel = True
                self.collected_count += 1
                self.stop_robot()
                self.state = State.NAVIGATING_TO_GREEN

    def _handle_nav_to_green(self):
        """Navigate to green zone."""
        if not self.is_navigation_active():
            self.get_logger().info('Going to GREEN zone...')
            self.send_nav_goal(self.green_zone_approx[0], self.green_zone_approx[1])
        else:
            # Check if we see green zone
            green = self.get_green_zones()
            if green and green[0].size > 500:
                self.get_logger().info('Green zone visible!')
                self.cancel_navigation()
                self.state = State.DELIVERING
                self.service_pending = False

    def _handle_delivering(self):
        """Deliver barrel at green zone."""
        green = self.get_green_zones()

        # Align with zone if visible
        if green:
            zone = green[0]
            x_offset = zone.x - self.IMAGE_CENTER_X  # Convert to offset from center
            if abs(x_offset) > 50:
                twist = Twist()
                twist.angular.z = -0.003 * x_offset
                self.cmd_vel_pub.publish(twist)
                return

        # Try offload
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                result = self.service_future.result()
                if result and result.success:
                    self.get_logger().info(f'DELIVERED! Total: {self.collected_count}')
                    self.has_barrel = False
                    self.stop_robot()
                    self.state = State.SEARCHING
                else:
                    # Move forward and retry
                    twist = Twist()
                    twist.linear.x = 0.1
                    self.cmd_vel_pub.publish(twist)
                self.service_pending = False
                self.service_future = None
            return

        # Start service call
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        self.service_future = self.offload_client.call_async(request)
        self.service_pending = True

    def _handle_nav_to_cyan(self):
        """Navigate to cyan zone for decontamination."""
        if not self.is_navigation_active():
            self.get_logger().info('Going to CYAN zone...')
            self.send_nav_goal(self.cyan_zone_approx[0], self.cyan_zone_approx[1])
        else:
            cyan = self.get_cyan_zones()
            if cyan and cyan[0].size > 500:
                self.get_logger().info('Cyan zone visible!')
                self.cancel_navigation()
                self.state = State.DECONTAMINATING
                self.service_pending = False

    def _handle_decontaminating(self):
        """Decontaminate at cyan zone."""
        cyan = self.get_cyan_zones()

        if cyan:
            zone = cyan[0]
            x_offset = zone.x - self.IMAGE_CENTER_X  # Convert to offset from center
            if abs(x_offset) > 50:
                twist = Twist()
                twist.angular.z = -0.003 * x_offset
                self.cmd_vel_pub.publish(twist)
                return

        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                result = self.service_future.result()
                if result and result.success:
                    self.get_logger().info('DECONTAMINATED!')
                    self.stop_robot()
                    if self.has_barrel:
                        self.state = State.NAVIGATING_TO_GREEN
                    else:
                        self.state = State.SEARCHING
                else:
                    twist = Twist()
                    twist.linear.x = 0.1
                    self.cmd_vel_pub.publish(twist)
                self.service_pending = False
                self.service_future = None
            return

        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        self.service_future = self.decontam_client.call_async(request)
        self.service_pending = True


def main(args=None):
    rclpy.init(args=args)
    node = CleanerBot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except ExternalShutdownException:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
