#!/bin/bash
# 在机器人正前方生成一个红色测试桶
# 用法: ./spawn_test_barrel.sh [x] [y] [name]
# 默认: x=0.0, y=-1.0 (机器人起始位置前方1米)

X=${1:-0.0}
Y=${2:--1.0}
NAME=${3:-test_barrel_$(date +%s)}

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{
  name: '$NAME',
  xml: '<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
    <model name=\"barrel\">
      <pose>0 0 0 0 0 0</pose>
      <static>false</static>
      <link name=\"body\">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.002645833</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.002645833</iyy>
            <iyz>0.0</iyz>
            <izz>0.001125</izz>
          </inertia>
        </inertial>
        <visual name=\"visual\">
          <geometry>
            <mesh>
              <uri>model://meshes/barrel_visual.stl</uri>
              <scale>0.005 0.005 0.005</scale>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>model://materials/assessment_materials.material</uri>
              <name>red_outlined</name>
            </script>
          </material>
        </visual>
        <collision name=\"visual\">
          <geometry>
            <mesh>
              <uri>model://meshes/barrel_collision.stl</uri>
              <scale>0.005 0.005 0.005</scale>
            </mesh>
          </geometry>
        </collision>
      </link>
    </model>
</sdf>',
  initial_pose: {
    position: {x: $X, y: $Y, z: 0.0}
  },
  reference_frame: 'world'
}"

echo "Spawned barrel '$NAME' at ($X, $Y)"
