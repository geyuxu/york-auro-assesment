# Barrel Collection Robot - Current Status and Issues

## Project Overview

This is a ROS2 Humble autonomous robot project for the AURO2025 assessment. The robot (TurtleBot3 Waffle Pi) must:
1. Search for red barrels in the environment
2. Collect them by driving through (barrel attaches to robot's back)
3. Deliver to green zone
4. Handle radiation by visiting cyan decontamination zone

## Current Implementation

### Architecture

- **Navigation**: Nav2 with AMCL localization using pre-built map
- **Visual Sensor**: C++ node (`assessment/src/visual_sensor.cpp`) detects colored objects
- **Control**: Python state machine (`solution/solution/cleaner_bot.py`)

### State Machine

```
IDLE -> SEARCHING -> APPROACHING -> RAMMING -> NAVIGATING_TO_GREEN -> DELIVERING
                                      |
                                      v
                          NAVIGATING_TO_CYAN -> DECONTAMINATING
```

### Key Files

| File | Description |
|------|-------------|
| `solution/solution/cleaner_bot.py` | Main controller (~570 lines) |
| `assessment/assessment/barrel_manager.py` | Pickup service backend |
| `assessment/src/visual_sensor.cpp` | Camera processing |
| `solution/launch/solution_launch.py` | Launch configuration |
| `solution/params/custom_nav2_params_namespaced.yaml` | Nav2 parameters |

## Visual Sensor Interface

The `visual_sensor` node publishes to `/robot1/barrels` topic:

```yaml
# assessment_interfaces/msg/Barrel.msg
uint8 colour    # 0=RED, 1=BLUE
float32 x       # Pixel X position (0-640), NOT offset from center!
float32 y       # Pixel Y position
float32 size    # Pixel area - larger = closer
```

**Critical**: `x` is absolute pixel coordinate. Image center is 320. To get offset:
```python
x_offset = barrel.x - 320  # negative=left, positive=right
angular_z = -0.006 * x_offset  # turn toward barrel
```

## Pickup Mechanism

From `barrel_manager.py` line ~180:
```python
def filter_points_behind(points, max_dist=0.45, alpha=np.deg2rad(30)):
```

**Barrel must be BEHIND the robot** within:
- Distance: < 0.45m
- Angle: within 30 degrees of directly behind

This means robot must **drive THROUGH the barrel** for pickup to succeed.

## Current Problems

### Problem 1: Wall Collision During Approach

**Symptom**: Robot sees barrel, rushes toward it, crashes into wall.

**Root Cause**: Camera can see barrel through doorways/openings, but LiDAR detects the wall in front.

**Current Solution** (partial):
```python
# In _handle_approaching()
if self.front_min_dist < 0.4:
    if best.size < 1500:  # Barrel should be bigger if it's really at 0.4m
        # It's a wall, not the barrel
        self.state = State.SEARCHING
        return
```

**Issue**: Thresholds (0.4m, 1500 pixels) are empirical and may not work in all cases.

### Problem 2: Wall Corner Collision

**Symptom**: Robot detects barrel on the side, turns toward it, hits wall corner.

**Current Solution**:
```python
# In _handle_searching()
if want_turn_left and self.left_min_dist < 0.35:
    return  # Don't turn, wall in the way
```

**Issue**: Still crashes sometimes. The 0.35m threshold may be too small.

### Problem 3: Getting Stuck

**Symptom**: Robot gets stuck in corner, keeps seeing barrel but can't reach it. Logs show repeated "ALIGNING" with large barrel sizes (10000+) but never succeeds.

**Current Solution**:
```python
# If barrel huge but stuck, go straight to ramming
if best.size > self.BARREL_SIZE_COMMIT:
    self.state = State.RAMMING
    return

# Detect stuck condition
if self.front_min_dist < 0.15 and self.left_min_dist < 0.25 and self.right_min_dist < 0.25:
    self.state = State.SEARCHING  # Abort
    return
```

**Issue**: Once stuck, Nav2 may not be able to escape. Need recovery behavior.

### Problem 4: Size-Distance Relationship Unknown

**Key Question**: What is the relationship between `barrel.size` (pixels) and actual distance?

Current empirical thresholds:
- `BARREL_SIZE_START = 500` - Start approaching
- `BARREL_SIZE_COMMIT = 3000` - Switch to ramming
- Size ~1500 at ~0.4m distance (guess)
- Size ~10000+ when very close

**Need**: Calibration data to establish reliable size-to-distance mapping.

## LiDAR Sectors

```python
# scan_callback() in cleaner_bot.py
front_ranges = msg.ranges[340:360] + msg.ranges[0:20]   # -20 to +20 deg
left_ranges = msg.ranges[20:70]                          # 20 to 70 deg
right_ranges = msg.ranges[290:340]                       # 290 to 340 deg
```

## Attempted Solutions That Failed

1. **Potential Field Controller**: Attraction from camera + repulsion from LiDAR. Failed because:
   - Camera gives pixel offset, not world coordinates
   - Hard to balance attraction vs repulsion gains

2. **Pure Visual Servoing**: Follow barrel using only camera. Failed because:
   - No obstacle awareness
   - Crashes into walls

3. **Nav2 for Everything**: Let Nav2 handle all navigation. Failed because:
   - Nav2 doesn't know about barrels
   - Can't guide robot to drive THROUGH barrel

## What Might Work Better

1. **Depth Camera**: If available, could give actual distance to barrel, not just pixel size.

2. **Better Size Calibration**: Measure actual size vs distance relationship empirically.

3. **Costmap Integration**: Add barrel detection to Nav2 costmap as a goal, not obstacle.

4. **Behavioral Approach**:
   - Use Nav2 until barrel is in clear line of sight
   - Only then switch to visual servoing
   - Define "clear line of sight" as: front_min_dist > barrel_estimated_distance

5. **Recovery Behaviors**: When stuck, back up and try different angle.

## Test Command

```bash
ros2 launch solution solution_launch.py use_nav2:=True
```

## Key Thresholds to Tune

| Parameter | Current Value | Location | Notes |
|-----------|--------------|----------|-------|
| BARREL_SIZE_START | 500 | cleaner_bot.py:101 | When to start approaching |
| BARREL_SIZE_COMMIT | 3000 | cleaner_bot.py:102 | When to start ramming |
| Wall detection front | 0.4m | cleaner_bot.py:456 | Front obstacle threshold |
| Wall corner threshold | 0.35m | cleaner_bot.py:373 | Side obstacle for turning |
| Stuck detection | 0.15m front, 0.25m sides | cleaner_bot.py:534 | All sides blocked |
| Steering gain | 0.006 | cleaner_bot.py:390 | P-controller for angular.z |

## Questions for Review

1. Is there a better way to distinguish "barrel in front" vs "wall in front with barrel visible behind it"?

2. Should we use Nav2 to navigate TO the barrel location (estimated from camera geometry)?

3. How to handle the case where barrel is visible but unreachable from current position?

4. Is there existing ROS2 code for visual servoing with obstacle avoidance that we're missing?

5. Should the barrel size thresholds be dynamic based on barrel position in frame (perspective)?
