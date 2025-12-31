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

from tf2_ros import Buffer, TransformListener, TransformException


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

        # TF2 for coordinate transforms (odom -> map)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State machine
        self.state = State.IDLE

        # Sensor data
        self.visible_barrels = []
        self.visible_zones = []
        self.current_radiation = 0.0
        # Position in MAP frame (from TF)
        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0
        self.position_valid = False  # Flag to track if we have valid position
        # LiDAR distances for corner protection
        self.front_min_dist = 10.0
        self.left_side_min_dist = 10.0
        self.right_side_min_dist = 10.0
        self.front_left_min_dist = 10.0
        self.front_right_min_dist = 10.0
        # Raw LiDAR ranges for distance lookup
        self.lidar_ranges = []

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
        self.BARREL_SIZE_COMMIT = 25000   # Switch to ramming - barrel must be VERY close
        self.IMAGE_CENTER_X = 320
        self.LINEAR_SPEED = 0.2
        self.ANGULAR_SPEED = 0.5
        self.SCAN_THRESHOLD = 0.35
        self.SCAN_STOP_THRESHOLD = 0.25
        self.CAMERA_FOV = 1.047  # ~60 degrees horizontal FOV

        # Approaching state
        self.approach_counter = 0
        self.MAX_APPROACH_ATTEMPTS = 100  # 10 seconds at 10Hz
        self.lost_sight_counter = 0

        # Ramming state
        self.ram_counter = 0
        self.MAX_RAM_ATTEMPTS = 100  # 10 seconds at 10Hz

        # Search waypoints - cover the map
        self.search_waypoints = [
           # (2, 3),
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
        # Get robot position in MAP frame using TF
        # Try multiple frame name variations
        frame_options = [
            f'{self.robot_name}/base_link',  # robot1/base_link
            'base_link',                      # Plain base_link
            f'{self.robot_name}/base_footprint',  # robot1/base_footprint
            'base_footprint',                 # Plain base_footprint
        ]

        tf_success = False
        for base_frame in frame_options:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.02)
                )
                self.map_x = trans.transform.translation.x
                self.map_y = trans.transform.translation.y

                q = trans.transform.rotation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                self.map_yaw = math.atan2(siny_cosp, cosy_cosp)
                self.position_valid = True
                tf_success = True
                break
            except TransformException:
                continue

        if not tf_success:
            # Fallback: use odom frame data directly
            # Note: This is in odom frame, not map frame!
            self.map_x = msg.pose.pose.position.x
            self.map_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.map_yaw = math.atan2(siny_cosp, cosy_cosp)
            # Mark position as valid if odom data is non-zero
            if abs(self.map_x) > 0.01 or abs(self.map_y) > 0.01:
                self.position_valid = True

    def radiation_callback(self, msg):
        for rad in msg.data:
            if rad.robot_id == self.robot_name:
                self.current_radiation = rad.level
                break

    def scan_callback(self, msg):
        if len(msg.ranges) < 360:
            return

        # Save raw ranges for distance lookup
        self.lidar_ranges = list(msg.ranges)

        def get_min_range(ranges):
            valid = [r for r in ranges if 0.01 < r < 10.0]
            return min(valid) if valid else 10.0

        # Front: -30 to +30 degrees (indices 330-359 and 0-30)
        front_ranges = list(msg.ranges[330:360]) + list(msg.ranges[0:30])
        self.front_min_dist = get_min_range(front_ranges)

        # Left side: 45 to 135 degrees (for corner protection)
        left_side_ranges = list(msg.ranges[45:135])
        self.left_side_min_dist = get_min_range(left_side_ranges)

        # Right side: 225 to 315 degrees (for corner protection)
        right_side_ranges = list(msg.ranges[225:315])
        self.right_side_min_dist = get_min_range(right_side_ranges)

        # Front-left: 30 to 60 degrees (for immediate corner detection)
        front_left_ranges = list(msg.ranges[30:60])
        self.front_left_min_dist = get_min_range(front_left_ranges)

        # Front-right: 300 to 330 degrees (for immediate corner detection)
        front_right_ranges = list(msg.ranges[300:330])
        self.front_right_min_dist = get_min_range(front_right_ranges)

    # ==================== HELPERS ====================

    def get_red_barrels(self):
        return [b for b in self.visible_barrels if b.colour == Barrel.RED]

    def get_green_zones(self):
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_GREEN]

    def get_cyan_zones(self):
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_CYAN]

    def estimate_barrel_distance(self, barrel_size):
        """Estimate distance to barrel from its pixel size.

        Calibrated values (rough estimates):
        - size 8000+ = ~0.2m (nearly fills screen, ramming distance)
        - size 5000  = ~0.4m
        - size 3000  = ~0.6m
        - size 1500  = ~1.0m
        - size 800   = ~1.5m
        - size 500   = ~2.0m
        - size 300   = ~3.0m
        """
        if barrel_size > 8000:
            return 0.2
        elif barrel_size > 5000:
            return 0.4
        elif barrel_size > 3000:
            return 0.6
        elif barrel_size > 1500:
            return 1.0
        elif barrel_size > 800:
            return 1.5
        elif barrel_size > 500:
            return 2.0
        elif barrel_size > 300:
            return 3.0
        else:
            return 5.0  # Very far

    def get_lidar_distance_at_angle(self, angle_deg):
        """Get LiDAR distance at a specific angle (degrees).

        LiDAR convention: 0° = front, positive = counter-clockwise (left)
        Index 0 = 0°, Index 90 = 90° (left), Index 270 = -90° (right)
        """
        if not self.lidar_ranges or len(self.lidar_ranges) < 360:
            return 2.0  # Default if no data

        # Normalize angle to 0-359
        idx = int(angle_deg) % 360

        # Get distance, check validity
        dist = self.lidar_ranges[idx]
        if 0.1 < dist < 10.0:
            return dist

        # If invalid, check nearby indices
        for offset in [1, -1, 2, -2, 5, -5]:
            nearby_idx = (idx + offset) % 360
            dist = self.lidar_ranges[nearby_idx]
            if 0.1 < dist < 10.0:
                return dist

        return 2.0  # Default

    def estimate_barrel_world_position(self, barrel):
        """Convert barrel camera detection to world coordinates (MAP frame).

        Uses LiDAR to measure actual distance to barrel, not pixel estimation!

        Camera: x=0 left, x=640 right, x=320 center
        LiDAR: 0° front, positive = left (counter-clockwise)
        """
        # Check if we have valid position data
        if not self.position_valid:
            self.get_logger().warn('No valid position data yet, cannot estimate barrel world position')
            return None

        # Calculate angle from camera pixel position
        # barrel.x < 320 = left side = positive angle
        # barrel.x > 320 = right side = negative angle
        x_offset = barrel.x - self.IMAGE_CENTER_X
        # Camera FOV ~62° for TurtleBot3, so half = 31°
        angle_in_camera = -x_offset * 31.0 / self.IMAGE_CENTER_X  # degrees

        # Get actual distance from LiDAR at that angle
        lidar_distance = self.get_lidar_distance_at_angle(angle_in_camera)

        # Transform to MAP frame
        world_angle = self.map_yaw + math.radians(angle_in_camera)
        world_x = self.map_x + lidar_distance * math.cos(world_angle)
        world_y = self.map_y + lidar_distance * math.sin(world_angle)

        self.get_logger().info(
            f'BARREL POS: robot=({self.map_x:.1f},{self.map_y:.1f}) yaw={math.degrees(self.map_yaw):.0f}° '
            f'cam_ang={angle_in_camera:.0f}° lidar_dist={lidar_distance:.1f}m -> ({world_x:.1f},{world_y:.1f})'
        )

        return (world_x, world_y)

    def is_wall_blocking_barrel(self, barrel):
        """Check if there's a wall between robot and barrel.

        Compare LiDAR distance to estimated visual distance.
        If LiDAR sees obstacle much closer than barrel should be,
        there's a wall in between.
        """
        visual_dist = self.estimate_barrel_distance(barrel.size)

        # Get front distance in barrel's direction
        x_offset = barrel.x - self.IMAGE_CENTER_X

        # Check the appropriate sector
        if abs(x_offset) < 50:
            # Barrel is roughly centered, check front
            lidar_dist = self.front_min_dist
        elif x_offset < 0:
            # Barrel is on left, check front-left
            lidar_dist = min(self.front_min_dist, self.front_left_min_dist)
        else:
            # Barrel is on right, check front-right
            lidar_dist = min(self.front_min_dist, self.front_right_min_dist)

        # If LiDAR sees obstacle significantly closer than barrel
        # there's likely a wall in the way
        if lidar_dist < (visual_dist - 0.4):
            self.get_logger().info(
                f'WALL BLOCKING: LiDAR={lidar_dist:.2f}m, barrel est={visual_dist:.2f}m'
            )
            return True

        return False

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

        # Log state occasionally (every second)
        if hasattr(self, '_loop_count'):
            self._loop_count += 1
        else:
            self._loop_count = 0

        if self._loop_count % 10 == 0:
            pos_status = "valid" if self.position_valid else "INVALID"
            self.get_logger().info(
                f'STATE: {self.state.name} pos=({self.map_x:.1f},{self.map_y:.1f}) [{pos_status}]'
            )

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
        """Search: Use Nav2 to patrol, watch for barrels.

        NEW STRATEGY: ALWAYS use Nav2 to navigate to barrel position.
        Only use visual servoing when barrel is VERY close (for final alignment).
        This ensures Nav2's obstacle avoidance is always active.
        """
        if self.has_barrel:
            return

        red_barrels = self.get_red_barrels()

        if red_barrels:
            # Get largest barrel
            best = max(red_barrels, key=lambda b: b.size)

            # Start approaching when barrel is visible and big enough
            if best.size > self.BARREL_SIZE_START:

                # If barrel is VERY big AND reasonably centered AND close, go straight to ramming
                if best.size > self.BARREL_SIZE_COMMIT:
                    x_offset = best.x - self.IMAGE_CENTER_X
                    # Also check LiDAR distance - must be within 0.8m to ram
                    if abs(x_offset) < 150 and self.front_min_dist < 0.8:
                        self.get_logger().info(f'BARREL HUGE! size={best.size:.0f} x={best.x:.0f} front={self.front_min_dist:.2f}m -> RAMMING')
                        self.cancel_navigation()
                        self.ram_counter = 0
                        self.lost_sight_counter = 0
                        self.state = State.RAMMING
                        return
                    else:
                        # Barrel is big but not centered or too far - go to APPROACHING
                        self.get_logger().info(f'BARREL HUGE but not ready: size={best.size:.0f} x={best.x:.0f} front={self.front_min_dist:.2f}m -> APPROACHING')
                        self.cancel_navigation()
                        self.approach_counter = 0
                        self.lost_sight_counter = 0
                        self.state = State.APPROACHING
                        return

                # If barrel is CLOSE (but not huge), switch to visual approach
                # This is the only time we use visual servoing - when very close
                if best.size > 3000:  # Close enough for visual servoing
                    x_offset = best.x - self.IMAGE_CENTER_X
                    if abs(x_offset) < 100:  # And reasonably centered
                        self.get_logger().info(f'BARREL CLOSE! size={best.size:.0f} -> APPROACHING')
                        self.cancel_navigation()
                        self.approach_counter = 0
                        self.lost_sight_counter = 0
                        self.state = State.APPROACHING
                        return

                # ALWAYS use Nav2 to navigate to barrel position
                # This ensures obstacle avoidance is handled by Nav2
                barrel_pos = self.estimate_barrel_world_position(best)
                if barrel_pos and not self.is_navigation_active():
                    self.get_logger().info(
                        f'BARREL SEEN size={best.size:.0f}! Nav2 to ({barrel_pos[0]:.1f}, {barrel_pos[1]:.1f})'
                    )
                    self.send_nav_goal(barrel_pos[0], barrel_pos[1])
                return

        # Continue patrol
        if not self.is_navigation_active():
            wp = self.search_waypoints[self.current_waypoint_idx]
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1}/{len(self.search_waypoints)}: ({wp[0]}, {wp[1]})')
            self.send_nav_goal(wp[0], wp[1])

    def _handle_approaching(self):
        """Visual servoing approach - ONLY called when path is clear.

        This state is entered only after confirming no wall is blocking.
        Simple strategy: keep barrel centered, move forward until close enough to ram.
        """
        self.approach_counter += 1

        # No timeout - keep approaching as long as we see the barrel

        red_barrels = self.get_red_barrels()

        if not red_barrels:
            self.lost_sight_counter += 1
            if self.lost_sight_counter > 30:  # 3 seconds
                self.get_logger().warn('Lost barrel, back to search')
                self.stop_robot()
                self.state = State.SEARCHING
                return
            self.stop_robot()
            return

        # Found barrel
        self.lost_sight_counter = 0
        best = max(red_barrels, key=lambda b: b.size)

        # Check if close enough to start ramming
        # Need both: large size AND close LiDAR distance
        if best.size > self.BARREL_SIZE_COMMIT and self.front_min_dist < 0.8:
            self.get_logger().info(f'BARREL CLOSE! size={best.size:.0f} front={self.front_min_dist:.2f}m -> RAMMING')
            self.ram_counter = 0
            self.lost_sight_counter = 0
            self.state = State.RAMMING
            return

        # Simple visual servoing: keep barrel centered, move forward at full speed
        x_offset = best.x - self.IMAGE_CENTER_X
        angular_z = -0.005 * x_offset
        angular_z = max(-0.5, min(0.5, angular_z))

        twist = Twist()
        twist.linear.x = 0.26  # Full speed during approach
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

        # Log occasionally
        if self.approach_counter % 10 == 0:
            self.get_logger().info(
                f'APPROACH: size={best.size:.0f} x={best.x:.0f} ang={angular_z:.2f}'
            )

    def _handle_ramming(self):
        """Ram: Drive forward to push through barrel, try pickup."""
        self.ram_counter += 1

        # Check if previous pickup call completed
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                result = self.service_future.result()
                if result and result.success:
                    self.get_logger().info(f'COLLECTED! {result.message}')
                    self.has_barrel = True
                    self.collected_count += 1
                    self.stop_robot()
                    self.state = State.NAVIGATING_TO_GREEN
                    self.service_pending = False
                    self.service_future = None
                    return
                self.service_pending = False
                self.service_future = None

        # No timeout - keep ramming as long as we see the barrel

        # Get barrel info
        red_barrels = self.get_red_barrels()

        twist = Twist()

        if red_barrels:
            self.lost_sight_counter = 0  # Reset lost sight counter
            best = max(red_barrels, key=lambda b: b.size)
            x_offset = best.x - self.IMAGE_CENTER_X

            # Log progress
            if self.ram_counter % 10 == 0:
                self.get_logger().info(
                    f'RAMMING: count={self.ram_counter} '
                    f'front={self.front_min_dist:.2f}m size={best.size:.0f} x={best.x:.0f}'
                )

            # Aggressive turning to keep barrel centered
            if abs(x_offset) > 150:
                # Barrel is far to the side - pure rotation, no forward
                twist.linear.x = 0.0
                twist.angular.z = 0.8 if x_offset < 0 else -0.8
            elif abs(x_offset) > 80:
                # Barrel is off-center - slow forward, strong turn
                twist.linear.x = 0.1
                twist.angular.z = 0.5 if x_offset < 0 else -0.5
            elif abs(x_offset) > 40:
                # Barrel is slightly off-center
                twist.linear.x = 0.2
                twist.angular.z = -0.004 * x_offset
            else:
                # Barrel is centered - full speed ahead!
                twist.linear.x = 0.26
                twist.angular.z = -0.003 * x_offset
        else:
            # Lost sight of barrel during ramming - likely passed through it!
            self.lost_sight_counter += 1

            if self.ram_counter % 10 == 0:
                self.get_logger().info(
                    f'RAMMING (blind): count={self.ram_counter} '
                    f'front={self.front_min_dist:.2f}m lost={self.lost_sight_counter}'
                )

            # If front is blocked (wall or we passed through barrel), keep trying pickup
            # The barrel might be behind us now - give more time for pickup to succeed
            if self.front_min_dist < 0.3:
                # Wall or obstacle ahead - stop and wait for pickup result
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # Give extra time when stopped (barrel likely behind us)
                if self.lost_sight_counter > 100:  # 10 seconds
                    self.get_logger().warn('Lost barrel during ramming (blocked), back to search')
                    self.stop_robot()
                    self.state = State.SEARCHING
                    return
            else:
                # Front is clear but can't see barrel - keep moving forward briefly
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                # Shorter timeout when moving forward blind
                if self.lost_sight_counter > 30:  # 3 seconds
                    self.get_logger().warn('Lost barrel during ramming (clear path), back to search')
                    self.stop_robot()
                    self.state = State.SEARCHING
                    return

        self.cmd_vel_pub.publish(twist)

        # Try pickup asynchronously (don't block!)
        if not self.service_pending:
            request = ItemRequest.Request()
            request.robot_id = self.robot_name
            self.service_future = self.pickup_client.call_async(request)
            self.service_pending = True

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
