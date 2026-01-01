import sys
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import BarrelList, ZoneList, Barrel, Zone, RadiationList
from auro_interfaces.srv import ItemRequest
from nav2_msgs.action import NavigateToPose


class State(Enum):
    """Robot state machine states"""
    IDLE = 0
    SEARCHING = 1
    APPROACHING_BARREL = 2
    COLLECTING_BARREL = 3
    NAVIGATING_TO_DROPOFF = 4
    DROPPING_OFF = 5
    NAVIGATING_TO_DECONTAMINATION = 6
    DECONTAMINATING = 7


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Get initial pose parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        # Get robot namespace
        self.robot_namespace = self.get_namespace()
        self.robot_name = self.robot_namespace.strip('/')

        self.get_logger().info(f"Robot {self.robot_name} initializing at x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")

        # State machine
        self.state = State.IDLE
        self.previous_state = State.IDLE

        # Sensor data
        self.visible_barrels = []
        self.visible_zones = []
        self.current_radiation_level = 0
        self.current_pose = None
        self.scan_data = None

        # Task tracking
        self.target_barrel = None
        self.has_barrel = False
        self.collected_barrels_count = 0
        self.search_waypoint_index = 0

        # Constants
        self.RADIATION_THRESHOLD = 80  # Decontaminate when radiation exceeds this
        self.BARREL_APPROACH_DISTANCE = 0.3  # meters
        self.ZONE_APPROACH_DISTANCE = 0.5  # meters
        self.LINEAR_SPEED = 0.2
        self.ANGULAR_SPEED = 0.3

        # Search waypoints (will be populated based on map exploration)
        self.search_waypoints = self._generate_search_waypoints()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.barrel_sub = self.create_subscription(
            BarrelList, 'barrels', self.barrel_callback, 10)
        self.zone_sub = self.create_subscription(
            ZoneList, 'zones', self.zone_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.radiation_sub = self.create_subscription(
            RadiationList, '/radiation_levels', self.radiation_callback, 10)

        # Service clients for barrel interaction
        self.pickup_client = self.create_client(ItemRequest, '/pick_up_item')
        self.dropoff_client = self.create_client(ItemRequest, '/offload_item')
        self.decontaminate_client = self.create_client(ItemRequest, '/decontaminate')

        # Wait for services
        self.get_logger().info("Waiting for barrel management services...")
        self.pickup_client.wait_for_service(timeout_sec=10.0)
        self.dropoff_client.wait_for_service(timeout_sec=10.0)
        self.decontaminate_client.wait_for_service(timeout_sec=10.0)

        # Nav2 action client (if using Nav2)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None

        # Publisher for initial pose (to set AMCL initial position)
        # DISABLED: Using AMCL's set_initial_pose parameter instead
        # self.initial_pose_pub = self.create_publisher(
        #     PoseWithCovarianceStamped, 'initialpose', 10)
        # self.initial_pose_timer = self.create_timer(5.0, self._publish_initial_pose)
        # self.initial_pose_retry_timer = None
        # self.initial_pose_attempts = 0
        # self.max_initial_pose_attempts = 20

        # Control loop timer
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.first_time = True
        self.get_logger().info(f"{self.robot_name} initialized and ready!")

    def _publish_initial_pose(self):
        """Publish initial pose to AMCL for localization, retry if needed"""
        # Cancel the initial delay timer
        if self.initial_pose_timer is not None:
            self.initial_pose_timer.cancel()
            self.initial_pose_timer = None

        # Check if we've exceeded max attempts
        self.initial_pose_attempts += 1
        if self.initial_pose_attempts > self.max_initial_pose_attempts:
            self.get_logger().warn("Max initial pose attempts reached, giving up")
            if self.initial_pose_retry_timer is not None:
                self.initial_pose_retry_timer.cancel()
            return

        # Convert yaw to quaternion
        qz = math.sin(self.initial_yaw / 2.0)
        qw = math.cos(self.initial_yaw / 2.0)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Set covariance (uncertainty)
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.07  # yaw variance

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"Published initial pose (attempt {self.initial_pose_attempts}): x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}")

        # Set up retry timer to keep publishing every 1 second
        if self.initial_pose_retry_timer is None:
            self.initial_pose_retry_timer = self.create_timer(1.0, self._publish_initial_pose)

    def _generate_search_waypoints(self):
        """Generate search waypoints for exploring the environment"""
        # These are approximate waypoints for the assessment environment
        # The robot will navigate to these to search for barrels
        waypoints = [
            (2.0, 0.0),    # Forward
            (2.0, 2.0),    # Right forward
            (0.0, 2.0),    # Right
            (-2.0, 2.0),   # Right back
            (-2.0, 0.0),   # Back
            (-2.0, -2.0),  # Left back
            (0.0, -2.0),   # Left
            (2.0, -2.0),   # Left forward
        ]
        return waypoints

    def barrel_callback(self, msg):
        """Handle barrel detections from visual sensor"""
        self.visible_barrels = msg.data

    def zone_callback(self, msg):
        """Handle zone detections from visual sensor"""
        self.visible_zones = msg.data

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Handle LiDAR scan data"""
        self.scan_data = msg

    def radiation_callback(self, msg):
        """Handle radiation level updates"""
        for radiation in msg.data:
            if radiation.robot_id == self.robot_name:
                self.current_radiation_level = radiation.level
                break

    def get_target_barrels(self):
        """Filter visible barrels to get only red ones"""
        return [b for b in self.visible_barrels if b.colour == Barrel.RED]

    def get_green_zones(self):
        """Filter visible zones to get only green (dropoff) zones"""
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_GREEN]

    def get_cyan_zones(self):
        """Filter visible zones to get only cyan (decontamination) zones"""
        return [z for z in self.visible_zones if z.zone == Zone.ZONE_CYAN]

    def call_service(self, client, robot_id):
        """Call a service and wait for response"""
        request = ItemRequest.Request()
        request.robot_id = robot_id

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f"Service call failed for {robot_id}")
            return None

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def drive_forward(self, speed=None):
        """Drive forward at specified speed"""
        if speed is None:
            speed = self.LINEAR_SPEED
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

    def rotate(self, angular_speed=None):
        """Rotate at specified angular speed"""
        if angular_speed is None:
            angular_speed = self.ANGULAR_SPEED
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

    def align_with_target(self, target_x):
        """Align robot with target using visual feedback
        target_x: pixel x-offset from camera center
        """
        # Camera typically has x=0 at center, positive to the right
        # We want to rotate to center the target

        threshold = 50  # pixel tolerance

        if abs(target_x) < threshold:
            # Aligned
            self.stop_robot()
            return True
        elif target_x > 0:
            # Target is to the right, rotate clockwise (negative angular)
            self.rotate(-self.ANGULAR_SPEED * 0.5)
        else:
            # Target is to the left, rotate counter-clockwise (positive angular)
            self.rotate(self.ANGULAR_SPEED * 0.5)

        return False

    def approach_target(self, target_size, target_distance):
        """Approach target based on its apparent size
        target_size: apparent size from visual sensor
        target_distance: desired distance to maintain
        """
        # Larger size means closer to target
        # This is a simple heuristic - adjust based on actual camera calibration

        size_threshold_min = 100  # Adjust based on testing
        size_threshold_max = 200

        if target_size < size_threshold_min:
            # Too far, move forward
            self.drive_forward(self.LINEAR_SPEED * 0.5)
            return False
        elif target_size > size_threshold_max:
            # Too close, stop
            self.stop_robot()
            return True
        else:
            # Good distance, stop
            self.stop_robot()
            return True

    def control_loop(self):
        """Main control loop - implements state machine"""

        if self.first_time:
            self.get_logger().info(f"{self.robot_name} starting autonomous operation")
            self.first_time = False
            self.state = State.SEARCHING

        # Check if we need to decontaminate (highest priority)
        if self.current_radiation_level > self.RADIATION_THRESHOLD and \
           self.state not in [State.DECONTAMINATING, State.NAVIGATING_TO_DECONTAMINATION]:
            self.get_logger().warn(f"Radiation level {self.current_radiation_level} exceeds threshold, going to decontaminate")
            self.previous_state = self.state
            self.state = State.NAVIGATING_TO_DECONTAMINATION

        # State machine logic
        if self.state == State.IDLE:
            self.stop_robot()

        elif self.state == State.SEARCHING:
            self._handle_searching()

        elif self.state == State.APPROACHING_BARREL:
            self._handle_approaching_barrel()

        elif self.state == State.COLLECTING_BARREL:
            self._handle_collecting_barrel()

        elif self.state == State.NAVIGATING_TO_DROPOFF:
            self._handle_navigating_to_dropoff()

        elif self.state == State.DROPPING_OFF:
            self._handle_dropping_off()

        elif self.state == State.NAVIGATING_TO_DECONTAMINATION:
            self._handle_navigating_to_decontamination()

        elif self.state == State.DECONTAMINATING:
            self._handle_decontaminating()

    def _handle_searching(self):
        """Search for red barrels"""
        red_barrels = self.get_target_barrels()

        if red_barrels and not self.has_barrel:
            # Found a barrel, switch to approaching
            self.target_barrel = red_barrels[0]  # Target the first visible barrel
            self.get_logger().info(f"Red barrel detected at x={self.target_barrel.x}, y={self.target_barrel.y}, size={self.target_barrel.size}")
            self.state = State.APPROACHING_BARREL
        else:
            # No barrel visible, continue searching
            # Simple search: rotate slowly to scan environment
            self.rotate(self.ANGULAR_SPEED * 0.3)

    def _handle_approaching_barrel(self):
        """Approach detected barrel"""
        if not self.target_barrel:
            self.state = State.SEARCHING
            return

        # First, align with barrel
        if not self.align_with_target(self.target_barrel.x):
            # Still aligning
            return

        # Then approach
        if self.approach_target(self.target_barrel.size, self.BARREL_APPROACH_DISTANCE):
            # Close enough, try to collect
            self.get_logger().info("Reached barrel, attempting collection")
            self.state = State.COLLECTING_BARREL

        # Update target barrel from sensor (it might move in view)
        red_barrels = self.get_target_barrels()
        if red_barrels:
            self.target_barrel = red_barrels[0]
        else:
            # Lost sight of barrel
            self.get_logger().warn("Lost sight of barrel during approach")
            self.target_barrel = None
            self.state = State.SEARCHING

    def _handle_collecting_barrel(self):
        """Collect the barrel"""
        # Drive into barrel
        self.drive_forward(self.LINEAR_SPEED * 0.3)

        # Call pickup service
        self.get_logger().info(f"Calling pickup service for {self.robot_name}")
        result = self.call_service(self.pickup_client, self.robot_name)

        if result and result.success:
            self.get_logger().info(f"Successfully collected barrel! Message: {result.message}")
            self.has_barrel = True
            self.collected_barrels_count += 1
            self.target_barrel = None
            self.stop_robot()
            self.state = State.NAVIGATING_TO_DROPOFF
        else:
            # Failed to collect, try again or search for another
            self.get_logger().warn(f"Failed to collect barrel: {result.message if result else 'No response'}")
            self.target_barrel = None
            self.state = State.SEARCHING

    def _handle_navigating_to_dropoff(self):
        """Navigate to green dropoff zone"""
        green_zones = self.get_green_zones()

        if green_zones:
            # Zone visible, approach it
            zone = green_zones[0]

            # Align with zone
            if not self.align_with_target(zone.x):
                return

            # Approach zone
            if zone.size > 150:  # Close enough (adjust threshold as needed)
                self.get_logger().info("Reached green zone, dropping off barrel")
                self.stop_robot()
                self.state = State.DROPPING_OFF
            else:
                # Move towards zone
                self.drive_forward(self.LINEAR_SPEED * 0.5)
        else:
            # No green zone visible, rotate to find it
            self.rotate(self.ANGULAR_SPEED * 0.3)

    def _handle_dropping_off(self):
        """Drop off the barrel at green zone"""
        self.get_logger().info(f"Calling dropoff service for {self.robot_name}")
        result = self.call_service(self.dropoff_client, self.robot_name)

        if result and result.success:
            self.get_logger().info(f"Successfully dropped off barrel! Total collected: {self.collected_barrels_count}. Message: {result.message}")
            self.has_barrel = False
            self.stop_robot()
            self.state = State.SEARCHING
        else:
            self.get_logger().warn(f"Failed to drop off barrel: {result.message if result else 'No response'}")
            # Stay in dropoff state to retry

    def _handle_navigating_to_decontamination(self):
        """Navigate to cyan decontamination zone"""
        cyan_zones = self.get_cyan_zones()

        if cyan_zones:
            # Zone visible, approach it
            zone = cyan_zones[0]

            # Align with zone
            if not self.align_with_target(zone.x):
                return

            # Approach zone
            if zone.size > 150:  # Close enough
                self.get_logger().info("Reached decontamination zone")
                self.stop_robot()
                self.state = State.DECONTAMINATING
            else:
                # Move towards zone
                self.drive_forward(self.LINEAR_SPEED * 0.5)
        else:
            # No cyan zone visible, rotate to find it
            self.rotate(self.ANGULAR_SPEED * 0.3)

    def _handle_decontaminating(self):
        """Decontaminate at cyan zone"""
        self.get_logger().info(f"Calling decontaminate service for {self.robot_name}")
        result = self.call_service(self.decontaminate_client, self.robot_name)

        if result and result.success:
            self.get_logger().info(f"Successfully decontaminated! Message: {result.message}")
            self.stop_robot()
            # Return to previous state
            if self.has_barrel:
                self.state = State.NAVIGATING_TO_DROPOFF
            else:
                self.state = State.SEARCHING
        else:
            self.get_logger().warn(f"Decontamination failed: {result.message if result else 'No response'}")
            # Stay in decontaminating state to retry

    def destroy_node(self):
        self.stop_robot()
        self.get_logger().info(f"{self.robot_name} shutting down. Total barrels collected: {self.collected_barrels_count}")
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
