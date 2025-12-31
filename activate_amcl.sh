#!/bin/bash
# Script to activate AMCL after launch
# Usage: ./activate_amcl.sh [robot_name]

ROBOT_NAME=${1:-robot1}

echo "Waiting for AMCL node to be available..."
sleep 5

echo "Configuring and activating AMCL for $ROBOT_NAME..."

# Source ROS2
source /opt/ros/humble/setup.bash
source /workspaces/AURO2025/install/setup.bash

# Configure and activate AMCL
ros2 lifecycle set /${ROBOT_NAME}/amcl configure
sleep 1
ros2 lifecycle set /${ROBOT_NAME}/amcl activate

echo "AMCL activated for $ROBOT_NAME"

# Optionally set initial pose
echo "Setting initial pose..."
ros2 service call /${ROBOT_NAME}/set_initial_pose nav2_msgs/srv/SetInitialPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 0.0, y: -2.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.717, w: 0.697}
      },
      covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.07]
    }
  }
}"

echo "Done! AMCL should now be publishing the map->odom transform."
