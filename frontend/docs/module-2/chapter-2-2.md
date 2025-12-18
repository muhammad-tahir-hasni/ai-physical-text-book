---
id: urdf-robot-description
title: Robot Description with URDF
sidebar_label: 2.2 Robot Description (URDF)
sidebar_position: 2
---

# Robot Description with URDF

## Introduction

The Unified Robot Description Format (URDF) is an XML specification for defining robot kinematics, dynamics, and visual properties. This chapter explores URDF structure, best practices, and conversion to SDF for Gazebo simulation.

## URDF Structure

A robot description consists of links (rigid bodies) and joints (connections):

```xml
<?xml version="1.0"?>
<robot name="mobile_robot">
  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheel Link -->
  <link name="left_wheel">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>

    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Joint Types

| Type | Description | Use Case |
|------|-------------|----------|
| **fixed** | No motion | Sensor mounts |
| **revolute** | Rotation with limits | Robot arms |
| **continuous** | Unlimited rotation | Wheels |
| **prismatic** | Linear motion | Linear actuators |
| **floating** | 6-DOF freedom | Free-flying robots |
| **planar** | 2D plane motion | Mobile bases |

## Inertia Calculation

Proper inertia values are critical for accurate physics:

```python
# Box inertia (mass m, dimensions x, y, z)
ixx = (m/12) * (y**2 + z**2)
iyy = (m/12) * (x**2 + z**2)
izz = (m/12) * (x**2 + y**2)

# Cylinder inertia (mass m, radius r, height h)
ixx = iyy = (m/12) * (3*r**2 + h**2)
izz = (m/2) * r**2

# Sphere inertia (mass m, radius r)
ixx = iyy = izz = (2/5) * m * r**2
```

## Xacro: URDF Macros

Xacro enables parametric robot descriptions with macros and properties:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_mass" value="1.0"/>

  <!-- Wheel Macro -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <inertial>
        <mass value="${wheel_mass}"/>
        <inertia ixx="${wheel_mass/12 * (3*wheel_radius**2 + wheel_width**2)}"
                 iyy="${wheel_mass/12 * (3*wheel_radius**2 + wheel_width**2)}"
                 izz="${wheel_mass/2 * wheel_radius**2}"
                 ixy="0" ixz="0" iyz="0"/>
      </inertial>

      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect * 0.2} -0.05" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>
</robot>
```

## Adding Sensors

### LiDAR

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
      </range>
    </lidar>
  </sensor>
</gazebo>
```

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.03 0.1 0.03"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
      </image>
    </camera>
  </sensor>
</gazebo>
```

## Gazebo-Specific Tags

```xml
<!-- Gazebo material -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>

<!-- Friction coefficients -->
<gazebo reference="left_wheel">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>

<!-- Differential drive plugin -->
<gazebo>
  <plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
  </plugin>
</gazebo>
```

## URDF to SDF Conversion

```bash
# Convert URDF to SDF
gz sdf -p robot.urdf > robot.sdf

# Check URDF validity
check_urdf robot.urdf

# Visualize URDF structure
urdf_to_graphiz robot.urdf
```

## Visualization in RViz

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro robot.urdf.xacro)"

# Launch joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## Best Practices

1. **Consistent Naming**: Use `<name>_link` and `<name>_joint` conventions
2. **Separate Collision/Visual**: Simplify collision meshes for performance
3. **Realistic Inertias**: Use physics-based calculations
4. **Proper Origins**: Set joint origins carefully for correct kinematics
5. **Use Xacro**: Parametrize repetitive elements
6. **Validate Early**: Check URDF before integrating with Gazebo

## Common Errors

```bash
# Error: Missing inertia
# Fix: Add inertial properties to all non-fixed links

# Error: "No inertial tag"
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>

# Error: Joint limits violated
# Fix: Set appropriate limits
<limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
```

## Summary

URDF provides a standard format for robot description, enabling portability across ROS 2 tools and simulators. Mastering URDF structure, Xacro macros, and Gazebo integration is essential for developing accurate robot simulations.

## References

1. [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
2. [Xacro Documentation](http://wiki.ros.org/xacro)
3. [Gazebo URDF Extensions](https://classic.gazebosim.org/tutorials?tut=ros_urdf)
4. [robot_state_publisher](https://github.com/ros/robot_state_publisher)
5. [SDF Format vs URDF](http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification)

---

**Previous**: [2.1 Gazebo Fundamentals](./chapter-2-1.md) | **Next**: Module 3 - NVIDIA Isaac Sim
