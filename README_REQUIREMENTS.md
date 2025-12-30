# AURO 2025 Assessment: Technical Requirements & Forum Clarifications

This document aggregates strict constraints, instructor clarifications, and known technical pitfalls from the module forum. It serves as the "Source of Truth" for the autonomous solution implementation.

## 1. Hard Technical Constraints (DO NOT VIOLATE)

### Forbidden Actions
* **NO Modification of Assessment Packages:** You strictly CANNOT modify `assessment`, `assessment_interfaces`, or the provided `visual_sensor` node.
* **NO Cheating with Sim Data:** You CANNOT use the `/model_states` topic. The robot must rely solely on its sensors (Lidar, Camera, Odom) and communication with other robots.
* **NO Teleoperation:** The final solution must be fully autonomous.

### Environment & Submission
* **Launch Entry Point:** Must use `solution_launch.py`.
    * Must launch `assessment.launch.py` internally.
    * Must accept all original arguments (though you can redirect them or add new ones).
* **DevContainer:**
    * Must use the `auro-vnc` configuration for final testing/submission.
    * Must list all `apt`/`pip` dependencies in `.devcontainer/devcontainer.json`.
    * **Must define 5 test scenarios** (with different `random_seed` values) in `devcontainer.json`.

## 2. Strategic Architecture (Based on Instructor Advice)

### Navigation Strategy: "Map First, Navigate Later"
* **The Issue:** Instructor noted `slam_toolbox` has issues with namespacing for multiple robots out-of-the-box.
* **The Approved Solution:**
    1.  **Pre-Mapping:** Run a single robot manually with SLAM to generate a static map (`.yaml` / `.pgm`) of the environment. Save this map.
    2.  **Localization:** Use `AMCL` (part of Nav2) with this static map for the assessment solution.
    3.  **Nav2 Config:** Launch Nav2 with the saved map.
    * *Note:* Forum reports indicate passing the map via command-line args to `solution_launch.py` can be buggy. **Hardcoding the default map path inside your launch file (or a robust params file) is recommended.**

### Dynamic vs. Static Elements
* **Barrels (Items):** Positions are **RANDOM** (controlled by `random_seed`).
    * *Requirement:* Robot must use coverage/search logic (e.g., frontier exploration or waypoints). Hardcoding barrel coordinates = 0 marks.
* **Zones:** Locations are generally static.
    * *Permission:* You can assume/approximate zone locations, BUT you should verify with Vision (Green/Blue detection) before interaction to be robust.

## 3. Operational Logic (The State Machine)

The robot(s) should implement a loop similar to:

1.  **Global Search:** Navigate through the map (using the static map for path planning) to find Red items.
2.  **Visual Confirmation:** Upon detecting Red pixels:
    * Interrupt Navigation.
    * Align with the object.
3.  **Collection (Service Call):**
    * Drive strictly into the item (required for pickup).
    * Call `Collect` service (check `assessment_interfaces`).
4.  **Delivery:**
    * Navigate to Green Zone coordinates.
    * Align/verify with Green pixels.
    * Call `Offload` service.
5.  **Self-Maintenance:**
    * Monitor contamination topic.
    * If high, divert to Blue Zone and wait/decontaminate.

## 4. Known Technical Pitfalls (From Forum)

* **Nav2 "False Success":** Nav2 may report "Goal Reached" when not actually there.
    * *Fix:* Implementation must double-check robot coordinates against the goal before assuming success.
* **Performance Timeouts:** Lab PCs may lag, causing `bt_navigator` to timeout.
    * *Fix:* Increase Nav2 timeout parameters in your config if possible, or ensure your logic is patient (retries).
* **Robot Speed:** Do not set `cmd_vel` to unrealistic speeds (e.g., > 1.0 m/s). It causes physics glitches.

## 5. Implementation Roadmap for AI Assistant

**Step 1: Map Generation (Manual/Scripted)**
* Create a script or instruction to launch the world + SLAM + Teleop to drive around and save the map (`ros2 run nav2_map_server map_saver_cli ...`).

**Step 2: Skeleton Package**
* Initialize `solution` package with `solution_launch.py`.
* Ensure `devcontainer.json` is updated.

**Step 3: Navigation Stack Integration**
* Create a launch file that starts `assessment.launch.py`, then starts `Nav2` (AMCL + Map Server) pointing to the saved map.

**Step 4: The Brain (Node)**
* Create a Python node `cleaner_bot.py`.
* Implement the State Machine.
* **Critical:** Subscribe to the visual sensor node (provided in assessment) to detect "Red", "Green", "Blue".

## 6. Verification Checklist (`rcutil.py`)

Before finishing, the code must pass:
* `./rcutil.py check-submission`
* No changes in restricted folders.
* Scenarios defined in JSON.