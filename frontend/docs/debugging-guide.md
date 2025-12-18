---
sidebar_position: 101
title: Debugging Guide
description: Common failure modes and troubleshooting strategies
keywords: [debugging, troubleshooting, failure modes, errors]
---

# Debugging Guide: Common Failure Modes

## Overview

This guide catalogs common failure modes across all four modules and provides systematic debugging strategies. Understanding what can go wrong helps you design more robust systems.

## Module 1: ROS 2 Communication Failures

### Failure Mode 1.1: Topic Name Mismatch

**Symptom**: Node publishes data but subscribers receive nothing. No error messages.

**Root Cause**: Publisher and subscriber use different topic names.

**Example**:
```
Publisher:  /robot/imu/data
Subscriber: /imu_data
```

**Debugging Steps**:
1. List all active topics: `ros2 topic list`
2. Check publisher topic: Verify name in publisher code
3. Check subscriber topic: Verify name in subscriber code
4. Ensure consistent naming conventions

**Solution**: Standardize on topic naming convention (e.g., `/robot_name/sensor_type/data_type`)

### Failure Mode 1.2: Message Type Incompatibility

**Symptom**: Runtime error: "Offered incompatible QoS - type mismatch"

**Root Cause**: Two nodes use different message types for same topic

**Example**:
```
Publisher:  sensor_msgs/Imu
Subscriber: geometry_msgs/PoseStamped
```

**Debugging Steps**:
1. Check topic info: `ros2 topic info /topic_name`
2. Verify expected message type in both nodes
3. Confirm message definitions match exactly

**Solution**: Ensure both nodes agree on message type

### Failure Mode 1.3: Communication Latency

**Symptom**: Control system oscillates or becomes unstable

**Root Cause**: Sensor data arrives too late for control decisions

**Debugging Steps**:
1. Measure actual latency: Record timestamp differences
2. Check Quality of Service (QoS) settings
3. Verify network bandwidth isn't saturated
4. Profile node execution time

**Solution**: Use reliable, low-latency QoS profiles for real-time data

## Module 2: Simulation Failures

### Failure Mode 2.1: Robot Falls Through Ground

**Symptom**: Robot drops infinitely downward, no ground contact

**Root Cause**: Missing or misconfigured collision geometry

**Debugging Steps**:
1. Check URDF for `<collision>` tags on all links
2. Verify collision mesh files exist and load correctly
3. Inspect Gazebo collision layers configuration
4. Enable collision visualization

**Solution**: Add proper collision geometry to URDF ground plane and robot links

### Failure Mode 2.2: Joints Behave Unrealistically

**Symptom**: Links fly apart, joints exceed limits, robot deforms

**Root Cause**: Physics parameters misconfigured or timestep too large

**Debugging Steps**:
1. Check joint limits in URDF
2. Verify joint damping and friction parameters
3. Reduce physics timestep (increase update rate)
4. Check for conflicting joint controllers

**Solution**: Use conservative physics settings, proper joint limits

### Failure Mode 2.3: Simulation Runs Slower Than Real-Time

**Symptom**: 1 second of simulation takes >1 second wall time

**Root Cause**: Physics computation exceeds available CPU

**Debugging Steps**:
1. Profile physics engine performance
2. Count active collision checks
3. Measure contact point computations
4. Check for excessive mesh complexity

**Solution**: Simplify collision meshes, reduce contact iterations, use convex hulls

## Module 3: Navigation Failures

### Failure Mode 3.1: SLAM Map Drift

**Symptom**: Robot position estimate diverges from true position over time

**Root Cause**: No loop closure, accumulated odometry errors

**Debugging Steps**:
1. Visualize estimated vs ground truth trajectory
2. Check for feature-rich environment (plain walls cause drift)
3. Verify loop closure detection is enabled
4. Inspect sensor noise parameters

**Solution**: Enable loop closure, add visual landmarks, improve sensor calibration

### Failure Mode 3.2: Path Planning Fails

**Symptom**: Nav2 reports "No path found" to valid goal

**Root Cause**: Obstacles block all paths, or goal is unreachable

**Debugging Steps**:
1. Visualize costmap to see obstacles
2. Verify goal position is in free space
3. Check inflation radius isn't too large
4. Ensure robot footprint is correctly defined

**Solution**: Adjust obstacle inflation, verify map accuracy, check goal feasibility

### Failure Mode 3.3: Robot Gets Stuck

**Symptom**: Robot stops moving, oscillates in place, or repeatedly backs up

**Root Cause**: Local minimum in navigation, conflicting costmaps

**Debugging Steps**:
1. Check local vs global costmap discrepancies
2. Verify recovery behaviors are configured
3. Inspect dynamic obstacle detection
4. Review planner parameters

**Solution**: Tune recovery behaviors, adjust planner patience parameters

## Module 4: VLA Pipeline Failures

### Failure Mode 4.1: Speech Recognition Error

**Symptom**: Whisper transcribes command incorrectly

**Root Cause**: Background noise, accent variation, unclear speech

**Debugging Steps**:
1. Test audio input quality
2. Check microphone noise levels
3. Try larger Whisper model for better accuracy
4. Test with sample audio files

**Solution**: Use noise cancellation, request clarification, provide visual confirmation

### Failure Mode 4.2: LLM Generates Infeasible Plan

**Symptom**: LLM outputs actions robot cannot execute

**Example**: "Fly to the ceiling" for ground-based robot

**Debugging Steps**:
1. Review LLM prompt engineering
2. Check if robot capabilities are specified in prompt
3. Verify constraint checking after plan generation
4. Test with diverse command set

**Solution**: Constrain LLM output with capability descriptions, validate plans before execution

### Failure Mode 4.3: Perception Cannot Find Object

**Symptom**: Object detection fails to locate requested item

**Root Cause**: Object not in field of view, lighting issues, occlusion

**Debugging Steps**:
1. Visualize camera view
2. Check object detection confidence scores
3. Verify object database includes target item
4. Test lighting conditions

**Solution**: Systematic search behavior, improved detection models, better lighting

## Systematic Debugging Approach

### Step 1: Reproduce the Failure

Create minimal test case that consistently triggers the failure

### Step 2: Isolate the Component

Test each module independently to identify which one fails

### Step 3: Check Interfaces

Verify all data flowing between components matches expected format

### Step 4: Inspect State

Log internal state variables before and after failure

### Step 5: Test Edge Cases

Try boundary conditions (empty environment, max sensor noise, etc.)

### Step 6: Verify Assumptions

Question assumptions about how system should behave

## Debugging Tools

### ROS 2 Command-Line Tools

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Show topic details
ros2 topic info /topic_name

# Display messages
ros2 topic echo /topic_name

# Visualize node graph
rqt_graph

# Record data for analysis
ros2 bag record -a
```

### Gazebo Debugging

- Enable collision visualization
- View joint states
- Monitor physics performance
- Inspect sensor outputs

### Nav2 Debugging

- Visualize costmaps in RViz
- Monitor path planning output
- Check transform tree
- Review recovery behavior logs

## Key Takeaways

- Most failures have systematic debugging procedures
- Understanding common failure modes accelerates troubleshooting
- Proper logging and visualization tools are essential
- Testing individual components before integration reduces complexity
- Clear error messages and fallback behaviors improve robustness

---

When designing your capstone project, consider each of these failure modes and how your system would detect and recover from them.
