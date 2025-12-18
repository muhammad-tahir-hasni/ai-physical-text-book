---
id: building-ros2-apps
title: Building ROS 2 Applications
sidebar_label: 1.2 Building Applications
sidebar_position: 2
---

# Building ROS 2 Applications

## Introduction

Moving from ROS 2 concepts to production-ready applications requires understanding package structure, build systems, and development workflows. This chapter demonstrates practical patterns for creating, organizing, and deploying ROS 2 software.

## Package Structure

A ROS 2 package is the fundamental unit of software organization:

```
my_robot_pkg/
├── package.xml          # Package metadata
├── CMakeLists.txt       # Build configuration (C++)
├── setup.py             # Build configuration (Python)
├── setup.cfg            # Python package config
├── resource/            # Package marker
├── my_robot_pkg/        # Python source
│   ├── __init__.py
│   ├── my_node.py
│   └── utils.py
├── src/                 # C++ source
│   └── my_cpp_node.cpp
├── include/             # C++ headers
├── launch/              # Launch files
│   └── robot.launch.py
├── config/              # Parameters (YAML)
│   └── robot_params.yaml
└── test/                # Unit tests
```

### Creating a Package

```bash
# Python package
ros2 pkg create --build-type ament_python my_py_pkg \
  --dependencies rclpy std_msgs

# C++ package
ros2 pkg create --build-type ament_cmake my_cpp_pkg \
  --dependencies rclcpp std_msgs
```

## Build Systems

### ament_python (Python Packages)

```python
# setup.py
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My robot package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
            'controller = my_robot_pkg.controller:main',
        ],
    },
)
```

### ament_cmake (C++ Packages)

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# Install targets
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

## Launch Files

Launch files orchestrate multiple nodes with parameters and remappings:

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')

    # Nodes
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'frame_id': 'camera_frame',
            'pixel_format': 'yuyv',
            'image_width': 640,
            'image_height': 480
        }]
    )

    detector_node = Node(
        package='my_robot_pkg',
        executable='object_detector',
        name='detector',
        parameters=[
            {'confidence_threshold': 0.7},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/camera/image_raw', '/camera/image')
        ]
    )

    return LaunchDescription([
        use_sim_time,
        camera_node,
        detector_node
    ])
```

Run with: `ros2 launch my_robot_pkg robot.launch.py use_sim_time:=true`

## Parameters

ROS 2 parameters enable runtime configuration without code changes:

### YAML Configuration

```yaml
# config/robot_params.yaml
/**:
  ros__parameters:
    robot_name: "MobileBot"
    max_speed: 1.5
    safety:
      obstacle_distance: 0.5
      emergency_stop_enabled: true
```

### Loading Parameters

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare parameters with defaults
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'default')

        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value

        # Parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                self.max_speed = param.value
                self.get_logger().info(f'Updated max_speed: {self.max_speed}')
        return SetParametersResult(successful=True)
```

## Component Composition

Components enable loading multiple nodes into a single process for efficiency:

```cpp
// C++ Component
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class MyComponent : public rclcpp::Node {
public:
  explicit MyComponent(const rclcpp::NodeOptions & options)
  : Node("my_component", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyComponent::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello from component";
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

Launch composed nodes:
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager my_pkg MyComponent
```

## Development Workflow

### Building Packages

```bash
# Build workspace
cd ~/ros2_ws
colcon build

# Build specific package
colcon build --packages-select my_robot_pkg

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing

```python
# test/test_my_node.py
import unittest
import rclpy
from my_robot_pkg.my_node import MyNode

class TestMyNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_initialization(self):
        node = MyNode()
        self.assertIsNotNone(node)
        self.assertEqual(node.get_name(), 'my_node')
        node.destroy_node()

if __name__ == '__main__':
    unittest.main()
```

Run tests: `colcon test --packages-select my_robot_pkg`

### Debugging

```bash
# Launch with GDB (C++)
ros2 run --prefix 'gdb -ex run --args' my_pkg my_node

# Launch with Python debugger
ros2 run --prefix 'python3 -m pdb' my_pkg my_node

# Enable debug logging
ros2 run my_pkg my_node --ros-args --log-level debug
```

## Best Practices

1. **Namespace Organization**: Group related packages in metapackages
2. **Interface Separation**: Define messages/services in separate packages
3. **Configuration Management**: Use parameters for environment-specific settings
4. **Testing**: Write unit tests for critical components
5. **Documentation**: Maintain README files and inline code documentation

## Common Patterns

### State Machine Node

```python
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    MOVING = 2
    CHARGING = 3

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.state = RobotState.IDLE
        self.timer = self.create_timer(0.1, self.state_callback)

    def state_callback(self):
        if self.state == RobotState.IDLE:
            self.handle_idle()
        elif self.state == RobotState.MOVING:
            self.handle_moving()
        # ... additional states
```

### Lifecycle Node (Managed)

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class LifecycleDemo : public rclcpp_lifecycle::LifecycleNode {
public:
  LifecycleDemo() : LifecycleNode("lifecycle_demo") {}

  CallbackReturn on_configure(const State &) override {
    RCLCPP_INFO(get_logger(), "Configuring...");
    // Setup resources
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const State &) override {
    RCLCPP_INFO(get_logger(), "Activating...");
    // Start publishing/processing
    return CallbackReturn::SUCCESS;
  }
};
```

## Summary

Building robust ROS 2 applications requires mastery of package structure, build systems, launch files, and component composition. By following established patterns and best practices, developers can create maintainable, testable, and scalable robotic systems.

## References

1. [ROS 2 Tutorials - Creating Packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
2. [ament Documentation](https://docs.ros.org/en/humble/Guides/Ament-CMake-Documentation.html)
3. [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
4. [Component Architecture](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
5. [ROS 2 Best Practices (Macenski, 2024)](https://navigation.ros.org/)

---

**Previous**: [1.1 ROS 2 Architecture](./chapter-1-1.md) | **Next**: Module 2 - Simulation
