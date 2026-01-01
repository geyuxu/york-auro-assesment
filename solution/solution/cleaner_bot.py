#!/usr/bin/env python3
"""
Autonomous Cleaner Bot - SIMPLIFIED VERSION

Strategy: Use Nav2 for ALL navigation, including approaching barrels.
Only use direct cmd_vel for the final "ramming" phase when very close.

State Machine:
1. SEARCHING: Nav2 patrol waypoints, watch for big red blobs
2. APPROACHING: Nav2 to barrel location (estimated from camera)
3. RAMMING: Direct forward motion to drive through barrel + pickup
4. NAVIGATING_TO_GREEN: Nav2 to delivery zone (GREEN)
5. DELIVERING: Call offload service to drop barrel
6. NAVIGATING_TO_CYAN: Nav2 to decontamination zone (CYAN)
7. DECONTAMINATING: Call decontaminate service
8. Back to SEARCHING
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node

# LLM interface for decision making
from . import llm_interface as llm
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, QoSPresetProfiles
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import BarrelList, ZoneList, Barrel, Zone, RadiationList
from auro_interfaces.srv import ItemRequest
from nav2_msgs.action import NavigateToPose

# TF imports removed - using AMCL pose directly instead
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class State(Enum):
    IDLE = auto()
    SEARCHING = auto()
    SCANNING = auto()     # 到达巡视点后旋转360°扫描
    APPROACHING = auto()  # Nav2 getting close to barrel
    RAMMING = auto()      # Direct forward to push through barrel
    RECOVER_LOCALIZATION = auto()  # Spin to fix AMCL after ramming
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

        # 直接订阅AMCL pose获取map坐标（比TF更可靠）
        # TF在namespaced环境下有问题，直接用AMCL pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.robot_namespace}/amcl_pose',
            self._amcl_pose_callback,
            10
        )

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
        self.rear_min_dist = 10.0  # For ramming backup phase
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

        # Stuck detection
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.stuck_counter = 0
        self.STUCK_THRESHOLD = 30  # 3 seconds at 10Hz without movement
        self.STUCK_DISTANCE = 0.1  # Must move at least 0.1m in 3 seconds
        self.recovery_counter = 0
        self.in_recovery = False
        self.recovery_phase = 0  # 0=turn to open space, 1=forward, 2=backup

        # Service call state
        self.service_future = None
        self.service_pending = False

        # Simple thresholds
        self.RADIATION_THRESHOLD = 50.0
        self.BARREL_SIZE_START = 2000     # Start approaching when barrel this big (closer)
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
        self.approach_target = None  # (x, y) 目标位置，进入APPROACHING时设置，不再更新

        # Ramming state
        self.ram_counter = 0
        self.ram_phase = 0  # 0=forward, 1=turning, 2=backup
        self.turn_counter = 0
        self.backup_counter = 0

        # Recover localization state (after ramming, spin to fix AMCL)
        self.recover_counter = 0
        self.recover_phase = 0  # 0=backup, 1=spin 360, 2=done
        self.MAX_RAM_ATTEMPTS = 100  # 10 seconds at 10Hz

        # Scanning state (左右扫描)
        self.scan_start_yaw = None
        self.scan_phase = 0  # 0=向左转, 1=回正, 2=向右转, 3=回正, 4=完成

        # 记录拾取桶时的巡视点索引，送完桶后返回
        self.waypoint_before_pickup = None

        # 标记是否已经开始导航（防止启动时原地扫描）
        self.has_started_navigation = False

        # Search waypoints - 手动测量值
        self.search_waypoints = [
             (1.5,-5.3),(-3.3,-11),(0,-12),(6.75,-11),(14,-11),(12.9,-17),(6,-15)
            # (2, 2),
            # (9.8, 6.0),
            # (10.0, 6.0),
            # (9.8, 9.0),
            # (9.8, 12.0),
            # (9.8, 15.0),
            # (7, 15.0),
            # (7, 12.0),
            # (7, 10.0),
            # (7, 9.0),
            # (14, 9.0),
            # (14, 15.0),
            # (9.5, -10.0),
            # (9.5, -20.0),
        ]

        # 导航失败计数器
        self.nav_fail_count = 0
        self.MAX_NAV_FAILS = 5  # 连续失败5次后跳过这个巡视点

        # 导航超时检测
        self.nav_start_time = None
        self.NAV_TIMEOUT = 180.0  # 180秒没到达就跳过这个巡视点

        # 防止重复发送导航目标
        self.last_nav_goal_time = None
        self.MIN_NAV_GOAL_INTERVAL = 2.0  # 至少间隔2秒才发送新目标

        # Zone locations (Gazebo世界坐标，地图已旋转对齐)
        # self.cyan_zone_approx = (7.5, 9.4)     # 消毒区
        # self.green_zone_approx = (13.5, 9.4)   # 交付区A
        self.cyan_zone_approx = (-9.3, -12.0)     # 消毒区
        self.green_zone_approx = (-15.0, -12.0)   # 交付区A

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

        # Dynamic mask parameter client (to disable rear LiDAR when carrying barrel)
        dynamic_mask_param_service = f'{self.robot_namespace}/dynamic_mask/set_parameters'
        self.dynamic_mask_param_client = self.create_client(
            SetParameters, dynamic_mask_param_service,
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

        # Costmap clear service clients
        self.clear_global_costmap_client = self.create_client(
            Empty, f'{self.robot_namespace}/global_costmap/clear_entirely_global_costmap',
            callback_group=self.service_callback_group)
        self.clear_local_costmap_client = self.create_client(
            Empty, f'{self.robot_namespace}/local_costmap/clear_entirely_local_costmap',
            callback_group=self.service_callback_group)

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

    def _amcl_pose_callback(self, msg):
        """Handle AMCL pose for map frame position."""
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.map_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.position_valid = True

    def odom_callback(self, msg):
        # 现在使用AMCL pose获取map坐标，odom回调只用于调试
        if not hasattr(self, '_odom_count'):
            self._odom_count = 0
        self._odom_count += 1
        if self._odom_count <= 3:
            self.get_logger().info(f'Odom received #{self._odom_count}: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')

        # 如果AMCL还没提供位置，用odom作为fallback（启动时）
        if not self.position_valid:
            self.map_x = msg.pose.pose.position.x
            self.map_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.map_yaw = math.atan2(siny_cosp, cosy_cosp)
            # 总是标记为valid，因为odom数据是可用的
            # 即使在原点也是有效位置
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

        # Rear: 150 to 210 degrees (behind robot, for ramming backup)
        rear_ranges = list(msg.ranges[150:210])
        self.rear_min_dist = get_min_range(rear_ranges)

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

    def _select_nearest_waypoint(self):
        """Select the nearest waypoint to continue from after delivery.

        This avoids inefficient routing by choosing the closest waypoint
        instead of returning to a distant one.
        """
        if not self.search_waypoints:
            return

        best_idx = 0
        best_dist = float('inf')

        for i, wp in enumerate(self.search_waypoints):
            dist = math.sqrt((self.map_x - wp[0])**2 + (self.map_y - wp[1])**2)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        self.current_waypoint_idx = best_idx
        self.get_logger().info(
            f'Selected nearest waypoint {best_idx + 1} at ({self.search_waypoints[best_idx][0]:.1f}, '
            f'{self.search_waypoints[best_idx][1]:.1f}), dist={best_dist:.1f}m'
        )

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

    def set_lidar_mask(self, sector: str):
        """Enable or disable LiDAR masking for specific sector.

        Args:
            sector: 'front' (330-30°), 'rear' (150-210°), or 'none' (disable)
        """
        if not self.dynamic_mask_param_client.service_is_ready():
            self.get_logger().warn('Dynamic mask param service not ready')
            return

        request = SetParameters.Request()

        if sector == 'front':
            # Mask front sector: 330° to 30° (barrel in front of robot)
            start_param = Parameter()
            start_param.name = 'ignore_sector_start'
            start_param.value = ParameterValue()
            start_param.value.type = ParameterType.PARAMETER_DOUBLE
            start_param.value.double_value = 330.0

            end_param = Parameter()
            end_param.name = 'ignore_sector_end'
            end_param.value = ParameterValue()
            end_param.value.type = ParameterType.PARAMETER_DOUBLE
            end_param.value.double_value = 30.0

            mask_param = Parameter()
            mask_param.name = 'mask_enabled'
            mask_param.value = ParameterValue()
            mask_param.value.type = ParameterType.PARAMETER_BOOL
            mask_param.value.bool_value = True

            request.parameters = [start_param, end_param, mask_param]
            self.get_logger().info('Enabling front LiDAR mask (330-30°) for barrel')

        elif sector == 'rear':
            # Mask rear sector: 120° to 240° (barrel behind robot, wider range)
            start_param = Parameter()
            start_param.name = 'ignore_sector_start'
            start_param.value = ParameterValue()
            start_param.value.type = ParameterType.PARAMETER_DOUBLE
            start_param.value.double_value = 120.0

            end_param = Parameter()
            end_param.name = 'ignore_sector_end'
            end_param.value = ParameterValue()
            end_param.value.type = ParameterType.PARAMETER_DOUBLE
            end_param.value.double_value = 240.0

            mask_param = Parameter()
            mask_param.name = 'mask_enabled'
            mask_param.value = ParameterValue()
            mask_param.value.type = ParameterType.PARAMETER_BOOL
            mask_param.value.bool_value = True

            request.parameters = [start_param, end_param, mask_param]
            self.get_logger().info('Enabling rear LiDAR mask (120-240°) for barrel')

        else:  # 'none' or any other value
            # Disable mask
            mask_param = Parameter()
            mask_param.name = 'mask_enabled'
            mask_param.value = ParameterValue()
            mask_param.value.type = ParameterType.PARAMETER_BOOL
            mask_param.value.bool_value = False

            request.parameters = [mask_param]
            self.get_logger().info('Disabling LiDAR mask')

        self.dynamic_mask_param_client.call_async(request)

    def clear_costmaps(self):
        """Clear both global and local costmaps."""
        self.get_logger().info('Clearing costmaps...')
        if self.clear_global_costmap_client.service_is_ready():
            self.clear_global_costmap_client.call_async(Empty.Request())
        if self.clear_local_costmap_client.service_is_ready():
            self.clear_local_costmap_client.call_async(Empty.Request())

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
        self.nav_start_time = self.get_clock().now()  # 记录导航开始时间
        return True

    def _nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav goal rejected!')
            self.nav_active = False
            self.nav_fail_count += 1
            # 连续失败太多次，跳过这个巡视点
            if self.nav_fail_count >= self.MAX_NAV_FAILS:
                self.get_logger().warn(f'Nav failed {self.nav_fail_count} times, skipping waypoint')
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
                self.nav_fail_count = 0
            return
        # 导航被接受，重置失败计数
        self.nav_fail_count = 0
        self.nav_goal_handle = goal_handle
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav succeeded!')
            # 标记导航成功到达，用于SEARCHING状态判断是否进入扫描
            self._nav_goal_reached = True
            self.nav_fail_count = 0  # 重置失败计数
        else:
            # 导航失败（被取消、超时、无法规划路径等）
            self.get_logger().warn(f'Nav failed with status: {result.status}')
            self.nav_fail_count += 1
            if self.nav_fail_count >= self.MAX_NAV_FAILS:
                self.get_logger().warn(f'Nav failed {self.nav_fail_count} times, skipping waypoint')
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
                self.nav_fail_count = 0
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

        # Priority: Radiation check (skip if already going to/at cyan, or going to green after decontam)
        if (self.current_radiation > self.RADIATION_THRESHOLD and
            self.state not in [State.NAVIGATING_TO_CYAN, State.DECONTAMINATING, State.NAVIGATING_TO_GREEN, State.DELIVERING]):
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

        # # Stuck detection and recovery (DISABLED - was causing issues)
        # if self.in_recovery:
        #     self.recovery_counter += 1
        #     twist = Twist()
        #
        #     # Phase 0: 转向开阔方向 (10 ticks = 1秒)
        #     if self.recovery_phase == 0:
        #         # 选择左前方或右前方更开阔的方向
        #         if self.front_left_min_dist > self.front_right_min_dist:
        #             twist.angular.z = 0.5  # 左转
        #         else:
        #             twist.angular.z = -0.5  # 右转
        #         twist.linear.x = 0.0
        #
        #         if self.recovery_counter > 10:
        #             self.get_logger().info('Recovery: turned, now moving forward')
        #             self.recovery_phase = 1
        #             self.recovery_counter = 0
        #
        #     # Phase 1: 向前走 (15 ticks = 1.5秒)
        #     elif self.recovery_phase == 1:
        #         twist.linear.x = 0.2
        #         twist.angular.z = 0.0
        #
        #         if self.recovery_counter > 15:
        #             self.get_logger().info('Recovery: forward done, checking if still stuck')
        #             self.recovery_phase = 2
        #             self.recovery_counter = 0
        #
        #     # Phase 2: 如果前方还是堵住，倒车 (15 ticks = 1.5秒)
        #     elif self.recovery_phase == 2:
        #         if self.front_min_dist < 0.4:
        #             # 前方还是堵住，倒车
        #             twist.linear.x = -0.2
        #             twist.angular.z = 0.0
        #             if self.recovery_counter > 15:
        #                 self.get_logger().info('Recovery complete (backed up)')
        #                 self.in_recovery = False
        #                 self.recovery_counter = 0
        #                 self.recovery_phase = 0
        #                 self.stuck_counter = 0
        #                 self.last_pos_x = self.map_x
        #                 self.last_pos_y = self.map_y
        #         else:
        #             # 前方开阔了，恢复完成
        #             self.get_logger().info('Recovery complete (path clear)')
        #             self.in_recovery = False
        #             self.recovery_counter = 0
        #             self.recovery_phase = 0
        #             self.stuck_counter = 0
        #             self.last_pos_x = self.map_x
        #             self.last_pos_y = self.map_y
        #
        #     self.cmd_vel_pub.publish(twist)
        #     return  # Skip normal state machine during recovery
        #
        # # Check if stuck (only during navigation states)
        # if self.state in [State.NAVIGATING_TO_GREEN, State.NAVIGATING_TO_CYAN, State.SEARCHING]:
        #     dist_moved = math.sqrt((self.map_x - self.last_pos_x)**2 + (self.map_y - self.last_pos_y)**2)
        #
        #     if dist_moved < self.STUCK_DISTANCE:
        #         self.stuck_counter += 1
        #         if self.stuck_counter > self.STUCK_THRESHOLD:
        #             self.get_logger().warn(f'STUCK detected! Trying to escape... (moved only {dist_moved:.2f}m in 3s)')
        #             self.cancel_navigation()
        #             self.in_recovery = True
        #             self.recovery_counter = 0
        #             self.recovery_phase = 0
        #     else:
        #         # Moving, reset stuck counter and update position
        #         self.stuck_counter = 0
        #         self.last_pos_x = self.map_x
        #         self.last_pos_y = self.map_y

        # State machine
        if self.state == State.IDLE:
            self.stop_robot()

        elif self.state == State.SEARCHING:
            self._handle_searching()

        elif self.state == State.SCANNING:
            self._handle_scanning()

        elif self.state == State.APPROACHING:
            self._handle_approaching()

        elif self.state == State.RAMMING:
            self._handle_ramming()

        elif self.state == State.RECOVER_LOCALIZATION:
            self._handle_recover_localization()

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

        # 看到桶就响应，不再等待到达第一个检查点
        if red_barrels:
            # 选择最佳桶：优先选择居中且较大的桶
            # 评分 = size - |x_offset| * 10 (居中的桶得分更高)
            def barrel_score(b):
                x_offset = abs(b.x - self.IMAGE_CENTER_X)
                return b.size - x_offset * 10
            best = max(red_barrels, key=barrel_score)

            # Start approaching when barrel is visible and big enough
            if best.size > self.BARREL_SIZE_START:

                # If barrel is VERY big AND reasonably centered AND close, go straight to ramming
                if best.size > self.BARREL_SIZE_COMMIT:
                    x_offset = best.x - self.IMAGE_CENTER_X
                    # Also check LiDAR distance - must be within 0.8m to ram
                    if abs(x_offset) < 150 and self.front_min_dist < 0.8:
                        self.get_logger().info(f'BARREL HUGE! size={best.size:.0f} x={best.x:.0f} front={self.front_min_dist:.2f}m -> RAMMING')
                        self.cancel_navigation()
                        self.set_lidar_mask('front')  # 屏蔽前方雷达，防止Nav2干扰
                        self.ram_counter = 0
                        self.ram_phase = 0
                        self.turn_counter = 0  # 重置旋转计数器
                        self.lost_sight_counter = 0
                        self.state = State.RAMMING
                        return
                    else:
                        # Barrel is big but not centered or too far - go to APPROACHING
                        barrel_pos = self.estimate_barrel_world_position(best)
                        if barrel_pos:
                            self.get_logger().info(f'BARREL HUGE but not ready: size={best.size:.0f} x={best.x:.0f} front={self.front_min_dist:.2f}m -> APPROACHING')
                            self.cancel_navigation()
                            self.approach_counter = 0
                            self.lost_sight_counter = 0
                            self.approach_target = barrel_pos
                            # 不屏蔽雷达，让Nav2正常避障
                            self.state = State.APPROACHING
                        return

                # If barrel is CLOSE (but not huge), switch to APPROACHING with Nav2
                if best.size > 3000:  # Close enough to approach
                    x_offset = best.x - self.IMAGE_CENTER_X
                    if abs(x_offset) < 100:  # And reasonably centered
                        barrel_pos = self.estimate_barrel_world_position(best)
                        if barrel_pos:
                            self.get_logger().info(f'BARREL CLOSE! size={best.size:.0f} -> APPROACHING')
                            self.cancel_navigation()
                            self.approach_counter = 0
                            self.lost_sight_counter = 0
                            self.approach_target = barrel_pos
                            # 不屏蔽雷达，让Nav2正常避障
                            self.state = State.APPROACHING
                        return

                # 看到桶就立即进入APPROACHING状态，取消当前导航
                barrel_pos = self.estimate_barrel_world_position(best)
                if barrel_pos:
                    self.get_logger().info(
                        f'BARREL SEEN size={best.size:.0f}! -> APPROACHING to ({barrel_pos[0]:.1f}, {barrel_pos[1]:.1f})'
                    )
                    self.cancel_navigation()
                    self.approach_counter = 0
                    self.lost_sight_counter = 0
                    self.approach_target = barrel_pos
                    # 不屏蔽雷达，让Nav2正常避障
                    self.state = State.APPROACHING
                return

        # Continue patrol
        # 检查导航超时 - 如果导航太久没到达，跳过这个巡视点
        if self.is_navigation_active() and self.nav_start_time is not None:
            elapsed = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
            if elapsed > self.NAV_TIMEOUT:
                wp = self.search_waypoints[self.current_waypoint_idx]
                self.get_logger().warn(f'Nav timeout ({elapsed:.0f}s) for waypoint {self.current_waypoint_idx + 1} ({wp[0]}, {wp[1]}), skipping!')
                self.cancel_navigation()
                self.clear_costmaps()  # 清除可能的错误障碍物
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
                self.nav_start_time = None
                return

        if not self.is_navigation_active():
            wp = self.search_waypoints[self.current_waypoint_idx]
            # 检查是否已经到达当前巡视点附近
            dist_to_wp = math.sqrt((self.map_x - wp[0])**2 + (self.map_y - wp[1])**2)

            # 必须满足三个条件才进入扫描：
            # 1. 距离巡视点足够近 (<1.0m)
            # 2. 已经发送过导航目标 (has_started_navigation)
            # 3. 导航目标已经到达（不是刚启动就在附近）
            if dist_to_wp < 1.0 and self.has_started_navigation and hasattr(self, '_nav_goal_reached'):
                # 已到达巡视点，进入SCANNING状态左右扫描
                self.get_logger().info(f'Arrived at waypoint {self.current_waypoint_idx + 1}, starting left-right scan')
                self.scan_start_yaw = self.map_yaw
                self.scan_phase = 0  # 从向左转开始
                self.scan_counter = 0  # 重置扫描计数器
                delattr(self, '_nav_goal_reached')  # 清除标记，下次需要重新导航
                self.state = State.SCANNING
            else:
                # 还没到，或者刚启动，先导航到巡检点
                # 防止重复发送目标（至少间隔2秒）
                now = self.get_clock().now()
                if self.last_nav_goal_time is None or \
                   (now - self.last_nav_goal_time).nanoseconds / 1e9 > self.MIN_NAV_GOAL_INTERVAL:
                    self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1}/{len(self.search_waypoints)}: ({wp[0]}, {wp[1]})')
                    self.send_nav_goal(wp[0], wp[1])
                    self.last_nav_goal_time = now
                    self.has_started_navigation = True  # 标记已开始导航

        # 墙壁避让：Nav2导航时，如果一侧靠近墙壁，稍微向另一侧转向
        if self.is_navigation_active():
            self._apply_wall_avoidance()

    def _apply_wall_avoidance(self):
        """根据左右两侧雷达距离，给小车一点转向速度避免蹭墙。

        条件：
        - 只有一侧靠近墙壁时才转向
        - 两侧都有墙或都没墙时不转向
        - 距离墙较远时不转向
        """
        WALL_CLOSE_THRESHOLD = 0.5   # 靠近墙壁的阈值
        TURN_SPEED = 0.15            # 转向速度

        left_dist = self.left_side_min_dist
        right_dist = self.right_side_min_dist

        left_close = left_dist < WALL_CLOSE_THRESHOLD
        right_close = right_dist < WALL_CLOSE_THRESHOLD

        twist = Twist()
        twist.linear.x = 0.0  # 不影响前进速度，Nav2控制

        # 只有一侧靠近墙壁时才转向
        if left_close and not right_close:
            # 左边有墙，向右转
            twist.angular.z = -TURN_SPEED
            self.cmd_vel_pub.publish(twist)
        elif right_close and not left_close:
            # 右边有墙，向左转
            twist.angular.z = TURN_SPEED
            self.cmd_vel_pub.publish(twist)
        # 两边都有墙或两边都远，不转向（让Nav2处理）

    def _handle_scanning(self):
        """SCANNING: 到达巡视点后左右扫描寻找红桶。

        扫描流程: 向左转20° -> 回正 -> 向右转20° -> 回正 -> 完成
        如果扫描过程中发现红桶，立即切换到APPROACHING。
        扫描完成后，前进到下一个巡视点。
        """
        SCAN_ANGLE = math.radians(25)  # 扫描角度25度

        # 扫描超时保护（20秒）
        if not hasattr(self, 'scan_counter'):
            self.scan_counter = 0
        self.scan_counter += 1
        if self.scan_counter > 200:  # 20秒超时
            self.get_logger().warn('SCANNING timeout, moving to next waypoint')
            self.scan_counter = 0
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
            self.state = State.SEARCHING
            return

        red_barrels = self.get_red_barrels()

        # 如果看到红桶，立即进入APPROACHING
        if red_barrels:
            # 优先选择居中且较大的桶
            def barrel_score(b):
                x_offset = abs(b.x - self.IMAGE_CENTER_X)
                return b.size - x_offset * 10
            best = max(red_barrels, key=barrel_score)
            if best.size > self.BARREL_SIZE_START:
                barrel_pos = self.estimate_barrel_world_position(best)
                if barrel_pos:
                    self.get_logger().info(f'SCANNING: Found barrel! size={best.size:.0f} x={best.x:.0f} -> APPROACHING')
                    self.approach_counter = 0
                    self.lost_sight_counter = 0
                    self.approach_target = barrel_pos
                    # 不屏蔽雷达
                    self.state = State.APPROACHING
                    return

        # 计算当前相对于起始位置的角度差
        yaw_diff = self.map_yaw - self.scan_start_yaw
        # 归一化到 -π 到 π
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi

        twist = Twist()

        # Phase 0: 向左转到+25度
        if self.scan_phase == 0:
            if yaw_diff < SCAN_ANGLE:
                twist.angular.z = 0.5  # 左转
            else:
                self.scan_phase = 1
                self.get_logger().info('SCANNING: Left scan done, returning to center')

        # Phase 1: 回正
        elif self.scan_phase == 1:
            if abs(yaw_diff) > 0.05:  # 约3度误差
                twist.angular.z = -0.4 if yaw_diff > 0 else 0.4
            else:
                self.scan_phase = 2
                self.get_logger().info('SCANNING: Centered, scanning right')

        # Phase 2: 向右转到-25度
        elif self.scan_phase == 2:
            if yaw_diff > -SCAN_ANGLE:
                twist.angular.z = -0.5  # 右转
            else:
                self.scan_phase = 3
                self.get_logger().info('SCANNING: Right scan done, returning to center')

        # Phase 3: 回正
        elif self.scan_phase == 3:
            if abs(yaw_diff) > 0.05:  # 约3度误差
                twist.angular.z = -0.4 if yaw_diff > 0 else 0.4
            else:
                self.scan_phase = 4
                self.get_logger().info('SCANNING: Scan complete, moving to next waypoint')

        # Phase 4: 完成，进入下一个巡视点
        if self.scan_phase == 4:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.search_waypoints)
            self.state = State.SEARCHING
            return

        self.cmd_vel_pub.publish(twist)

    def _handle_approaching(self):
        """APPROACHING: 使用Nav2导航到桶的位置，避免撞墙。

        目标位置在进入APPROACHING时设置(approach_target)，不会更新。
        Nav2会自动避障。当桶足够大且距离足够近时，切换到RAMMING。
        如果Nav2认为到达但距离还远，使用视觉伺服直接前进。
        """
        self.approach_counter += 1

        # APPROACHING超时：如果300 ticks (30秒) 还没进入RAMMING，放弃
        MAX_APPROACH_TICKS = 300
        if self.approach_counter > MAX_APPROACH_TICKS:
            self.get_logger().warn(f'APPROACHING timeout ({self.approach_counter} ticks), giving up')
            self.cancel_navigation()
            self.approach_target = None
            self.set_lidar_mask('none')
            self.state = State.SEARCHING
            return

        red_barrels = self.get_red_barrels()

        # 检查是否可以进入RAMMING
        if red_barrels:
            self.lost_sight_counter = 0
            best = max(red_barrels, key=lambda b: b.size)

            # 检查是否足够近可以RAMMING
            # 必须: 桶可见且非常大(>25000) AND LiDAR距离足够近(<0.8m)
            if best.size > self.BARREL_SIZE_COMMIT and self.front_min_dist < 0.8:
                self.get_logger().info(f'BARREL VERY CLOSE! size={best.size:.0f} front={self.front_min_dist:.2f}m -> RAMMING')
                self.cancel_navigation()
                self.approach_target = None  # 清除目标
                # 保持前方mask，APPROACHING已经启用了，不要取消
                self.ram_counter = 0
                self.ram_phase = 0
                self.turn_counter = 0  # 重置旋转计数器
                self.lost_sight_counter = 0
                self.state = State.RAMMING
                return

            # Nav2认为到达但距离还远 -> 使用视觉伺服直接前进
            if not self.is_navigation_active() and self.front_min_dist > 0.4:
                # 视觉伺服：边转边走，使用比例控制
                x_offset = best.x - self.IMAGE_CENTER_X
                twist = Twist()

                # 角速度：比例控制
                twist.angular.z = -0.004 * x_offset
                # 限制角速度
                twist.angular.z = max(-0.6, min(0.6, twist.angular.z))

                # 线速度：始终保持较高速度，只有极端偏移时才减速
                if abs(x_offset) > 250:
                    # 极端偏移，中速前进+快速转向
                    twist.linear.x = 0.18
                else:
                    # 正常情况，全速前进
                    twist.linear.x = 0.26

                self.cmd_vel_pub.publish(twist)

                if self.approach_counter % 10 == 0:
                    self.get_logger().info(f'APPROACHING: Visual servo, barrel_size={best.size:.0f} x_offset={x_offset:.0f} front={self.front_min_dist:.2f}m')
                return

        else:
            self.lost_sight_counter += 1
            # 丢失视野太久，可能桶被推开或者根本不是桶
            # 不要轻易进入RAMMING，返回SEARCHING重新寻找
            if self.lost_sight_counter > 30:  # 3秒看不到桶
                self.get_logger().warn(f'Lost sight for too long ({self.lost_sight_counter} ticks) -> back to SEARCHING')
                self.cancel_navigation()
                self.approach_target = None
                self.set_lidar_mask('none')
                self.state = State.SEARCHING
                return

        # 使用Nav2导航到目标位置（只发送一次）
        if self.approach_target and not self.is_navigation_active():
            self.get_logger().info(f'APPROACHING: Nav2 to target ({self.approach_target[0]:.1f}, {self.approach_target[1]:.1f})')
            self.send_nav_goal(self.approach_target[0], self.approach_target[1])

        # Log occasionally
        if self.approach_counter % 10 == 0:
            barrel_info = ""
            if red_barrels:
                best = max(red_barrels, key=lambda b: b.size)
                barrel_info = f" barrel_size={best.size:.0f}"
            self.get_logger().info(
                f'APPROACHING: count={self.approach_counter} front={self.front_min_dist:.2f}m nav_active={self.nav_active}{barrel_info}'
            )

    def _handle_ramming(self):
        """Ram: 向前靠近 → 刹停 → 旋转180度 → 刹停 → 向后倒车 + 持续尝试pickup

        Phases:
        0: 向前靠近直到足够近 (front_min_dist < 0.4m, 约1.5个机器人直径)
        1: 刹停 (10个tick确保线速度完全为0)
        2: 旋转180度 (约40个tick)
        3: 刹停 (10个tick确保角速度完全为0)
        4: 向后倒车并持续尝试pickup (不转向)

        超时机制: Phase 2超过500 ticks或Phase 4超过300 ticks，放弃当前桶
        """
        self.ram_counter += 1

        # 超时检测：如果Phase 2旋转太久(50秒)或Phase 4倒车太久(30秒)，放弃
        MAX_PHASE2_TICKS = 500  # 50秒
        MAX_PHASE4_TICKS = 300  # 30秒

        if self.ram_phase == 2 and self.ram_counter > MAX_PHASE2_TICKS:
            self.get_logger().warn(f'RAMMING: Phase 2 timeout ({self.ram_counter} ticks), abandoning barrel')
            self.stop_robot()
            self.set_lidar_mask('none')
            self.clear_costmaps()  # 清除RAMMING期间积累的错误障碍物
            if hasattr(self, 'ram_start_yaw'):
                del self.ram_start_yaw
            if hasattr(self, 'ram_last_yaw'):
                del self.ram_last_yaw
            if hasattr(self, 'ram_total_rotated'):
                del self.ram_total_rotated
            self.state = State.SEARCHING
            return

        if self.ram_phase == 4 and self.ram_counter > MAX_PHASE4_TICKS:
            self.get_logger().warn(f'RAMMING: Phase 4 timeout ({self.ram_counter} ticks), abandoning barrel')
            self.stop_robot()
            self.set_lidar_mask('none')
            self.clear_costmaps()  # 清除RAMMING期间积累的错误障碍物
            self.state = State.SEARCHING
            return

        # Check if previous pickup call completed
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                result = self.service_future.result()
                if result and result.success:
                    self.get_logger().info(f'COLLECTED! {result.message}')

                    # 检查是否拿到了错误的桶（蓝桶）
                    # barrel_manager的消息格式: "Robot 'robot1' picked up barrel 'barrel_X' successfully"
                    # 红桶是 CONTAMINATED 类型，蓝桶是 OTHER 类型
                    # 但消息中不包含颜色信息，我们需要另一种方式判断
                    # 简单方案：检查后方是否还能看到红桶（如果能看到说明拿错了）
                    # 但旋转180度后摄像头朝向已经变了...
                    # 最安全的方案：直接送去green zone，让系统自动判断

                    self.has_barrel = True
                    self.collected_count += 1
                    self.stop_robot()
                    # Enable rear LiDAR mask to ignore barrel behind robot
                    self.set_lidar_mask('rear')
                    # 记录当前巡视点，送完桶后返回
                    self.waypoint_before_pickup = self.current_waypoint_idx
                    # Go to RECOVER first to fix AMCL localization after ramming
                    self.recover_counter = 0
                    self.recover_phase = 0
                    self.state = State.RECOVER_LOCALIZATION
                    self.get_logger().info(f'Pickup success! -> RECOVER_LOCALIZATION (will return to waypoint {self.current_waypoint_idx + 1})')
                    self.service_pending = False
                    self.service_future = None
                    return
                # pickup失败，继续尝试
                self.service_pending = False
                self.service_future = None

        twist = Twist()

        # Log progress
        if self.ram_counter % 10 == 0:
            # Phase 4时显示rear距离（倒车撞桶），其他阶段显示front
            if self.ram_phase == 4:
                self.get_logger().info(
                    f'RAMMING: phase={self.ram_phase} count={self.ram_counter} '
                    f'rear={self.rear_min_dist:.2f}m yaw={math.degrees(self.map_yaw):.1f}°'
                )
            else:
                self.get_logger().info(
                    f'RAMMING: phase={self.ram_phase} count={self.ram_counter} '
                    f'front={self.front_min_dist:.2f}m yaw={math.degrees(self.map_yaw):.1f}°'
                )

        # Phase 0: 向前靠近红桶直到足够近 (约1.5个机器人直径)
        # 使用视觉伺服保持对准红桶，避免撞到蓝桶
        if self.ram_phase == 0:
            red_barrels = self.get_red_barrels()
            if self.front_min_dist > 0.4:
                twist.linear.x = 0.1  # 慢速向前

                # 如果能看到红桶，保持对准
                if red_barrels:
                    best_red = max(red_barrels, key=lambda b: b.size)
                    x_offset = best_red.x - self.IMAGE_CENTER_X
                    # 用角速度修正方向，保持对准红桶
                    if abs(x_offset) > 30:
                        twist.angular.z = -0.3 if x_offset > 0 else 0.3
                    else:
                        twist.angular.z = 0.0
                else:
                    twist.angular.z = 0.0
            else:
                self.get_logger().info(f'Close enough (front={self.front_min_dist:.2f}m), stopping')
                self.ram_phase = 1
                self.ram_counter = 0
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        # Phase 1: 刹停 (确保速度完全为0)
        elif self.ram_phase == 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.ram_counter >= 10:  # 刹停1秒确保完全停止
                self.get_logger().info('Stopped, start turning 180°')
                self.ram_phase = 2
                self.turn_counter = 0

        # Phase 2: 旋转180度 (基于累积角度检测)
        elif self.ram_phase == 2:
            self.turn_counter += 1
            twist.linear.x = 0.0
            twist.angular.z = 0.8  # 左转 (逆时针)

            # 记录起始角度和上一次角度（第一次进入时）
            if not hasattr(self, 'ram_start_yaw'):
                self.ram_start_yaw = self.map_yaw
                self.ram_last_yaw = self.map_yaw
                self.ram_total_rotated = 0.0
                self.get_logger().info(f'RAMMING: Start rotation, initial yaw={math.degrees(self.ram_start_yaw):.1f}°')

            # 计算本次增量旋转（处理角度环绕）
            delta_yaw = self.map_yaw - self.ram_last_yaw
            # 归一化到 -π 到 π
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            # 累加旋转量（逆时针为正）
            self.ram_total_rotated += delta_yaw
            self.ram_last_yaw = self.map_yaw

            rotated_deg = abs(math.degrees(self.ram_total_rotated))

            # 每10个tick记录一次旋转进度
            if self.turn_counter % 10 == 0:
                self.get_logger().info(f'RAMMING: Rotating... rotated={rotated_deg:.1f}° yaw={math.degrees(self.map_yaw):.1f}°')

            # 检查是否旋转了约180度
            if rotated_deg > 170:  # 接近180度
                self.get_logger().info(f'Finished turning 180°, rotated={rotated_deg:.1f}° final yaw={math.degrees(self.map_yaw):.1f}°')
                # 转完180度后，桶在后方，切换mask从front到rear
                self.set_lidar_mask('rear')
                del self.ram_start_yaw  # 清除起始角度
                del self.ram_last_yaw
                del self.ram_total_rotated
                self.ram_phase = 3
                self.ram_counter = 0  # 重置计数器用于刹停阶段

        # Phase 3: 刹停 (确保角速度完全为0再倒车)
        elif self.ram_phase == 3:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.ram_counter >= 10:  # 刹停1秒确保完全停止
                self.get_logger().info('Rotation stopped, now backing up to barrel')
                self.ram_target_yaw = self.map_yaw  # 记录目标航向，用于保持直线倒车
                self.ram_phase = 4
                self.ram_counter = 0  # 重置计数器

        # Phase 4: 向后倒车 (已旋转180°，桶在背后) 并持续尝试pickup
        elif self.ram_phase == 4:
            twist.linear.x = -0.15  # 倒车撞桶

            # 强制航向保持，忽略墙壁避让（靠rear mask屏蔽桶）
            if hasattr(self, 'ram_target_yaw'):
                yaw_error = self.map_yaw - self.ram_target_yaw
                # 归一化到 -π 到 π
                while yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                while yaw_error < -math.pi:
                    yaw_error += 2 * math.pi

                # 强航向保持，增益1.0
                twist.angular.z = -1.0 * yaw_error
                # 限制角速度
                twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
            else:
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

        # Phase 4时持续尝试pickup
        if self.ram_phase == 4 and not self.service_pending:
            request = ItemRequest.Request()
            request.robot_id = self.robot_name
            self.service_future = self.pickup_client.call_async(request)
            self.service_pending = True

    def _handle_recover_localization(self):
        """Recover localization after ramming.

        RAMMING involves 180° turn + backup which causes odometry drift.
        This state spins the robot to let AMCL see room features and correct position.

        Phases:
        0: Move forward slightly to get away from wall/barrel (15 ticks = 1.5s)
        1: Spin 360° to let AMCL resample particles (60 ticks = 6s at 0.5 rad/s ≈ 360°)
        2: Stop and transition to NAVIGATING_TO_CYAN (decontamination first)
        """
        self.recover_counter += 1
        twist = Twist()

        # Log progress
        if self.recover_counter % 10 == 0:
            self.get_logger().info(
                f'RECOVER_LOCALIZATION: phase={self.recover_phase} count={self.recover_counter}'
            )

        # Phase 0: Move forward slightly to get away from obstacles
        if self.recover_phase == 0:
            twist.linear.x = 0.1
            twist.angular.z = 0.0

            if self.recover_counter >= 15:  # 1.5 seconds
                self.get_logger().info('RECOVER: Forward done, starting 360° spin')
                self.recover_phase = 1
                self.recover_counter = 0

        # Phase 1: Spin 360° to let AMCL see room walls
        elif self.recover_phase == 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Slow spin for better AMCL sampling

            # 360° at 0.5 rad/s = ~12.6 seconds, but 6s is usually enough
            if self.recover_counter >= 60:  # 6 seconds
                self.get_logger().info('RECOVER: Spin complete, localization should be fixed')
                self.recover_phase = 2

        # Phase 2: Stop and transition
        elif self.recover_phase == 2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # 清除costmap（AMCL定位修复后，基于正确位置重建）
            self.clear_costmaps()
            # 先去绿区放桶，再去消毒
            self.get_logger().info('RECOVER complete -> NAVIGATING_TO_GREEN (deliver first)')
            self.green_nav_counter = 0
            self.state = State.NAVIGATING_TO_GREEN

        self.cmd_vel_pub.publish(twist)

    def _handle_nav_to_green(self):
        """Navigate to green zone."""
        # 初始化导航超时计数器
        if not hasattr(self, 'green_nav_counter'):
            self.green_nav_counter = 0

        self.green_nav_counter += 1

        # 检查是否能看到 green zone（主要判断条件）
        green = self.get_green_zones()
        if green and green[0].size > 300:  # 降低阈值，更容易检测到
            self.get_logger().info(f'Green zone visible! size={green[0].size:.0f} -> DELIVERING')
            self.cancel_navigation()
            self.green_nav_counter = 0
            self.state = State.DELIVERING
            self.service_pending = False
            return

        # 检查是否在绿区半径内（0.8米）
        if self.position_valid:
            dx = self.map_x - self.green_zone_approx[0]
            dy = self.map_y - self.green_zone_approx[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 0.8:
                self.get_logger().info(f'Within GREEN zone radius! dist={dist:.2f}m -> DELIVERING')
                self.cancel_navigation()
                self.green_nav_counter = 0
                self.state = State.DELIVERING
                self.service_pending = False
                return

        # 防止Nav2假成功导致无限循环
        if self.green_nav_counter > 6000:  # 600秒(10分钟)超时
            self.get_logger().warn('Green nav timeout (10min), forcing DELIVERING state')
            self.cancel_navigation()
            self.green_nav_counter = 0
            self.state = State.DELIVERING
            self.service_pending = False
            return

        # 继续导航
        if not self.is_navigation_active():
            # 每次重发导航目标前清除costmap
            self.clear_costmaps()
            if self.green_nav_counter % 50 == 1:
                self.get_logger().info(f'Going to GREEN zone... counter={self.green_nav_counter}')
            self.send_nav_goal(self.green_zone_approx[0], self.green_zone_approx[1])

    def _handle_delivering(self):
        """Deliver barrel at green zone - 进入绿区就放下桶."""
        # 检查服务调用结果
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                result = self.service_future.result()
                if result and result.success:
                    self.get_logger().info(f'DELIVERED! Total: {self.collected_count}')
                    self.has_barrel = False
                    self.stop_robot()
                    self.set_lidar_mask('none')
                    # 放完桶后去消毒
                    self.cyan_nav_counter = 0
                    self.get_logger().info('Barrel delivered -> NAVIGATING_TO_CYAN (decontaminate)')
                    self.state = State.NAVIGATING_TO_CYAN
                self.service_pending = False
                self.service_future = None
            return

        # 直接调用offload服务
        self.get_logger().info('Calling /offload_item service...')
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        self.service_future = self.offload_client.call_async(request)
        self.service_pending = True

    def _handle_nav_to_cyan(self):
        """Navigate to cyan zone for decontamination."""
        # 初始化导航超时计数器
        if not hasattr(self, 'cyan_nav_counter'):
            self.cyan_nav_counter = 0

        self.cyan_nav_counter += 1

        # 检查是否能看到 cyan zone（主要判断条件）
        cyan = self.get_cyan_zones()
        if cyan and cyan[0].size > 300:  # 降低阈值，更容易检测到
            self.get_logger().info(f'Cyan zone visible! size={cyan[0].size:.0f} -> DECONTAMINATING')
            self.cancel_navigation()
            self.cyan_nav_counter = 0
            self.state = State.DECONTAMINATING
            self.service_pending = False
            return

        # 检查是否在消毒区半径内（0.8米）
        if self.position_valid:
            dx = self.map_x - self.cyan_zone_approx[0]
            dy = self.map_y - self.cyan_zone_approx[1]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 0.8:
                self.get_logger().info(f'Within CYAN zone radius! dist={dist:.2f}m -> DECONTAMINATING')
                self.cancel_navigation()
                self.cyan_nav_counter = 0
                self.state = State.DECONTAMINATING
                self.service_pending = False
                return

        # 防止Nav2假成功导致无限循环：如果导航很快就"成功"但没看到zone，继续等待
        # 最多等待6000个tick（600秒=10分钟），之后强制进入DECONTAMINATING尝试调用服务
        if self.cyan_nav_counter > 6000:  # 600秒(10分钟)超时
            self.get_logger().warn('Cyan nav timeout (10min), forcing DECONTAMINATING state')
            self.cancel_navigation()
            self.cyan_nav_counter = 0
            self.state = State.DECONTAMINATING
            self.service_pending = False
            return

        # 继续导航（即使Nav2报成功也继续发送，直到看到zone或超时）
        if not self.is_navigation_active():
            # 每次重发导航目标前清除costmap
            self.clear_costmaps()
            if self.cyan_nav_counter % 50 == 1:
                self.get_logger().info(f'Going to CYAN zone... counter={self.cyan_nav_counter}')
            self.send_nav_goal(self.cyan_zone_approx[0], self.cyan_zone_approx[1])

    def _handle_decontaminating(self):
        """Decontaminate at cyan zone."""
        # 初始化重试计数器
        if not hasattr(self, 'decontam_retry_count'):
            self.decontam_retry_count = 0

        # 检查服务调用结果
        if self.service_pending and self.service_future is not None:
            if self.service_future.done():
                result = self.service_future.result()
                if result and result.success:
                    self.stop_robot()
                    self.decontam_retry_count = 0  # 重置计数器
                    # 消毒成功，继续搜索下一个桶
                    self.get_logger().info('DECONTAMINATED! -> SEARCHING')
                    self._select_nearest_waypoint()
                    self.waypoint_before_pickup = None
                    self.state = State.SEARCHING
                else:
                    # 消毒失败，记录并重试
                    self.decontam_retry_count += 1
                    msg = result.message if result else "No result"
                    self.get_logger().warn(f'Decontaminate failed ({self.decontam_retry_count}): {msg}')

                    # 如果重试超过10次，强制继续搜索
                    if self.decontam_retry_count >= 10:
                        self.get_logger().warn('Decontaminate failed 10 times, proceeding to SEARCHING anyway')
                        self.decontam_retry_count = 0
                        self._select_nearest_waypoint()
                        self.waypoint_before_pickup = None
                        self.state = State.SEARCHING
                    else:
                        # 向前移动一点再重试
                        twist = Twist()
                        twist.linear.x = 0.1
                        self.cmd_vel_pub.publish(twist)
                self.service_pending = False
                self.service_future = None
            return

        # 发起服务调用
        self.get_logger().info(f'Calling /decontaminate service... (retry={self.decontam_retry_count})')
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
