# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is the AURO2025 robotics course repository containing practicals and an autonomous robot assessment. The project uses ROS2 Humble with TurtleBot3 Waffle Pi robots in Gazebo simulation. The core task is developing an autonomous system that collects colored barrels and delivers them to designated zones.

## Key Commands

### Workspace Management
```bash
# Build the workspace
colcon build --symlink-install

# Clean workspace
./rcutil.py clean

# Install dependencies from devcontainer.json
./rcutil.py install-dependencies
```

### Running & Testing
```bash
# Launch solution
ros2 launch solution solution_launch.py

# Launch with specific parameters
ros2 launch solution solution_launch.py num_robots:=2 random_seed:=42 use_nav2:=true

# List available scenarios
./rcutil.py list-scenarios

# Run specific scenario
./rcutil.py run-scenario "Scenario 1"

# Check submission before packaging
./rcutil.py check-submission

# Create submission ZIP
./rcutil.py zip-workspace
```

### Development Tools
```bash
# Show launch arguments
ros2 launch solution solution_launch.py --show-args

# List launch files and their arguments
./rcutil.py list-launch

# Monitor topics
ros2 topic echo /robot1/barrels
ros2 topic echo /barrel_log
ros2 topic echo /radiation_levels
```

## Architecture

### Package Structure

**Restricted Packages (DO NOT MODIFY):**
- `assessment/`: Simulation environment and nodes
- `assessment_interfaces/`: Message/service definitions for assessment
- `gazebo_ros_link_attacher/`: Plugin for barrel attachment
- `auro_interfaces/srv/ItemRequest.srv`: Service definition for barrel operations

**Development Packages:**
- `solution/`: Primary development package for autonomous solution
- `solution_interfaces/`: Custom message definitions (create as needed)
- Additional ROS2 packages can be created as necessary
- `week_1/` through `week_10/`: Practice materials and examples

### Critical Files

**Launch System:**
- [solution/launch/solution_launch.py](solution/launch/solution_launch.py): Main entry point, must include [assessment/launch/assessment.launch.py](assessment/launch/assessment.launch.py) and support all existing parameters
- [solution/config/initial_poses.yaml](solution/config/initial_poses.yaml): Robot spawn positions for 1-3 robots
- [solution/params/custom_nav2_params_namespaced.yaml](solution/params/custom_nav2_params_namespaced.yaml): Nav2 configuration

**Dev Container:**
- [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json): Must use `auro-vnc` config for submission, must define 5 test scenarios with different `random_seed` values, list all apt/pip dependencies

**Validation:**
- [rcutil.py](rcutil.py): Workspace utility for building, testing, and packaging

### ROS2 Node Architecture

**Environment Nodes (barrel_manager):**
- Services: `/pick_up_item`, `/offload_item`, `/decontaminate` (all use `auro_interfaces/srv/ItemRequest`)
- Topics: `/barrel_log` (BarrelLog), `/barrel_holders` (BarrelHolders), `/radiation_levels` (RadiationList)

**Per-Robot Nodes (namespaced as /robotX/):**
- `visual_sensor`: Processes camera to detect colored barrels and zones
  - Topics: `barrels` (BarrelList), `zones` (ZoneList)
  - Debug topics (if `vision_sensor_debug:=true`): `camera/image_barrels`, `camera/image_zones`, `camera/image_zones_mask`
- `dynamic_mask`: Filters LiDAR data by angle range
  - Input: `scan`, Output: `scan_filtered`
  - Parameters: `ignore_sector_start`, `ignore_sector_end`, `mask_enabled`

**Solution Nodes:**
- `robot_controller`: Autonomous control logic (implement state machine here)
- `data_logger`: Logs barrel collection and radiation metrics to CSV

### Navigation Strategy

**Recommended Approach (from instructor guidance):**
1. **Pre-mapping Phase**: Use single robot with SLAM to generate static map
   ```bash
   # Save map after SLAM mapping
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```
2. **Localization**: Use AMCL with the pre-generated static map (avoid slam_toolbox multi-robot issues)
3. **Navigation**: Launch Nav2 with saved map via `use_nav2:=true` parameter

**Important Notes:**
- Barrel positions are RANDOM (controlled by `random_seed`), cannot be hardcoded
- Zone positions are static but should verify with vision before interaction
- Robot must use sensor data only - `/model_states` topic is forbidden

## Implementation Patterns

### Typical State Machine Flow

1. **Search**: Navigate environment using Nav2 to find red barrels
2. **Visual Confirmation**: Detect red pixels from visual_sensor, align with barrel
3. **Collection**: Drive into barrel, call `/pick_up_item` service with robot identifier
4. **Delivery**: Navigate to green zone, verify with green pixels, call `/offload_item`
5. **Maintenance**: Monitor `/radiation_levels`, decontaminate at blue zone if needed

### Service Call Example
```python
from auro_interfaces.srv import ItemRequest

# Create service client
client = self.create_client(ItemRequest, '/pick_up_item')

# Call service
request = ItemRequest.Request()
request.robot_name = 'robot1'
future = client.call_async(request)
```

### Message Interfaces
- `assessment_interfaces/msg/Barrel.msg`: x/y offset, size, color
- `assessment_interfaces/msg/BarrelList.msg`: List of visible barrels
- `assessment_interfaces/msg/Zone.msg`: x/y offset, size, color
- `assessment_interfaces/msg/ZoneList.msg`: List of visible zones

## Known Technical Issues

- **Nav2 False Success**: Goal reached callback may fire before robot arrives - verify position independently
- **Performance Timeouts**: Increase Nav2 timeout parameters in custom_nav2_params_namespaced.yaml if needed
- **Physics Glitches**: Keep cmd_vel linear.x <= 1.0 m/s, angular.z <= 1.0 rad/s
- **Map Path Bug**: Hardcode map path in launch file rather than passing via command-line args

## Launch Parameters

**Critical Parameters:**
- `num_robots`: Number of robots (1-3)
- `random_seed`: Controls random barrel distribution
- `experiment_duration`: Simulation time limit in seconds (default: 840)
- `use_nav2`: Enable Nav2 navigation stack
- `sensor_noise`: Enable realistic sensor noise
- `odometry_source`: WORLD or ENCODER
- `use_rviz`: Launch RViz visualization
- `headless`: Run without Gazebo GUI
- `limit_real_time_factor`: Limit to real-time (false = faster simulation)

**Vision Sensor Parameters:**
- `vision_sensor_debug`: Publish debug image topics
- `vision_sensor_skip_frames`: Skip similar frames (default: true)
- `vision_sensor_frame_divider`: Fraction of frames to process (default: 0.5)

## Dev Container Configurations

- `auro-vnc`: VNC desktop at http://localhost:6080/ (REQUIRED for submission)
- `auro-wsl`: WSL2 with native Windows GUI support
- `auro-linux`: X11 for native Linux Docker

## Testing & Submission

1. Define 5 scenarios in `.devcontainer/devcontainer.json` with different `random_seed` values
2. Run `./rcutil.py check-submission` to verify:
   - No modifications to restricted packages
   - Scenarios properly defined
   - Launch file supports all required parameters
3. Package with `./rcutil.py zip-workspace`
