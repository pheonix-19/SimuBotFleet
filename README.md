# SimuBotFleet - ROS 2 Differential Drive Robot Simulation and Control

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy-blue.svg)](https://docs.ros.org/en/rolling/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

SimuBotFleet is a comprehensive ROS 2 workspace designed for simulating and controlling differential drive robots. It provides a complete ecosystem for robot development, including simulation in Gazebo, real hardware interfaces, localization, and control systems. The project supports both simulated and real robot operations with extensive examples and tutorials.

## Project Structure

```
SimuBotFleet/
├── src/
│   ├── simubot_bringup/          # Main launch configurations
│   ├── simubot_controller/       # Robot control and odometry
│   ├── simubot_description/      # URDF models and visualization
│   ├── simubot_localization/     # Robot localization using EKF
│   ├── simubot_msgs/             # Custom message definitions
│   ├── simubot_py_examples/      # Python ROS 2 examples
│   └── simubot_cpp_examples/     # C++ ROS 2 examples
├── build/                        # Build artifacts
├── install/                      # Installation directory
└── log/                          # Build and runtime logs
```

## Package Descriptions

### 🚀 simubot_bringup
**Main orchestration package for launching complete robot systems**

- **Purpose**: Provides high-level launch files for different deployment scenarios
- **Key Features**:
  - Simulated robot launch configuration
  - Real robot hardware interface launch
  - Integrated sensor and control system startup

### 🎮 simubot_controller
**Robot control, kinematics, and odometry package**

- **Purpose**: Implements differential drive robot control and odometry calculation
- **Key Features**:
  - **Simple Controller**: Basic differential drive controller for velocity commands
  - **Noisy Controller**: Advanced controller with sensor noise simulation for realistic behavior
  - **Dual Implementation**: Both Python and C++ versions available
  - **Odometry Publishing**: Real-time robot position and velocity estimation
  - **Joystick Teleoperation**: Manual robot control using game controllers
  - **Wheel Kinematics**: Forward and inverse kinematics for differential drive robots

**Controller Parameters**:
- `wheel_radius`: 0.033m (default)
- `wheel_separation`: 0.17m (default)
- `wheel_radius_error`: 0.005m (for noisy controller)
- `wheel_separation_error`: 0.02m (for noisy controller)

### 🤖 simubot_description
**Robot model definition and visualization package**

- **Purpose**: Contains URDF/Xacro robot models and visualization configurations
- **Key Features**:
  - **URDF Model**: Complete robot description with meshes and physics properties
  - **Gazebo Integration**: Simulation-ready robot model with sensors
  - **RViz Configuration**: Pre-configured visualization setup
  - **Multi-platform Support**: Compatible with Gazebo Classic and Gazebo Ignition/Garden
  - **Sensor Integration**: IMU, wheel encoders, and other sensor models

### 📍 simubot_localization
**Robot state estimation and localization package**

- **Purpose**: Provides accurate robot pose estimation using sensor fusion
- **Key Features**:
  - **Extended Kalman Filter (EKF)**: Fuses wheel odometry and IMU data
  - **IMU Integration**: Processes accelerometer and gyroscope data
  - **Transform Publishing**: Maintains accurate coordinate frame relationships
  - **Noise Handling**: Robust estimation despite sensor noise

### 📨 simubot_msgs
**Custom message and service definitions**

- **Purpose**: Defines custom interfaces for robot communication
- **Key Features**:
  - Custom message types for robot-specific data
  - Service definitions for robot operations
  - Standardized communication protocols

### 🐍 simubot_py_examples
**Python ROS 2 learning examples and tutorials**

- **Purpose**: Comprehensive examples for learning ROS 2 concepts in Python
- **Available Examples**:
  - **simple_publisher.py**: Basic topic publishing
  - **simple_subscriber.py**: Topic subscription and message handling
  - **simple_service_server.py**: Service server implementation
  - **simple_service_client.py**: Service client usage
  - **simple_parameter.py**: Parameter handling and dynamic reconfiguration
  - **simple_lifecycle_node.py**: Managed node lifecycle implementation
  - **simple_tf_kinematics.py**: Transform operations and coordinate frames
  - **simple_turtlesim_kinematics.py**: Turtle robot kinematics example

### ⚡ simubot_cpp_examples
**C++ ROS 2 learning examples and tutorials**

- **Purpose**: High-performance examples for learning ROS 2 concepts in C++
- **Available Examples**:
  - **simple_publisher.cpp**: Efficient topic publishing
  - **simple_subscriber.cpp**: Fast message processing
  - **simple_service_server.cpp**: High-performance service implementation
  - **simple_service_client.cpp**: Asynchronous service calls
  - **simple_parameter.cpp**: Parameter management
  - **simple_lifecycle_node.cpp**: Managed node lifecycle
  - **simple_tf_kinematics.cpp**: Transform calculations
  - **simple_turtlesim_kinematics.cpp**: Robot kinematics implementation

## Installation and Setup

### Prerequisites

- **ROS 2**: Humble, Iron, or Jazzy distribution
- **Gazebo**: Garden or Ignition (depending on ROS 2 version)
- **Dependencies**: All required packages are specified in package.xml files

### Building the Workspace

```bash
# Clone the repository
git clone https://github.com/pheonix-19/SimuBotFleet.git
cd SimuBotFleet

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Usage and Operation

### 🎮 Running Simulated Robot

Launch the complete simulated robot with Gazebo physics simulation:

```bash
# Launch simulated robot with Gazebo, controller, and joystick
ros2 launch simubot_bringup simulated_robot.launch.py

# Alternative: Launch individual components
ros2 launch simubot_description gazebo.launch.py
ros2 launch simubot_controller controller.launch.py use_simple_controller:=false use_python:=false
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=true
```

### 🔧 Running Real Robot

Launch the real robot hardware interface:

```bash
# Launch complete real robot system
ros2 launch simubot_bringup real_robot.launch.py

# Alternative: Launch individual components
ros2 launch simubot_firmware hardware_interface.launch.py  # (requires simubot_firmware package)
ros2 launch simubot_controller controller.launch.py use_simple_controller:=false use_python:=false use_sim_time:=false
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=false
```

### 📊 Visualization and Monitoring

#### RViz Visualization
```bash
# Launch robot model visualization with GUI controls
ros2 launch simubot_description display.launch.py

# Launch robot model without GUI (for headless systems)
ros2 launch simubot_description display_no_gui.launch.py
```

#### Robot Localization
```bash
# Launch EKF-based localization
ros2 launch simubot_localization local_localization.launch.py use_python:=false
```

### 🎛️ Controller Configuration Options

The controller system provides flexible configuration options:

```bash
# Use simple controller (basic functionality)
ros2 launch simubot_controller controller.launch.py use_simple_controller:=true

# Use noisy controller (realistic sensor simulation)
ros2 launch simubot_controller controller.launch.py use_simple_controller:=false

# Use Python implementation
ros2 launch simubot_controller controller.launch.py use_python:=true

# Use C++ implementation (default, better performance)
ros2 launch simubot_controller controller.launch.py use_python:=false

# Custom wheel parameters
ros2 launch simubot_controller controller.launch.py \
    wheel_radius:=0.035 \
    wheel_separation:=0.18 \
    wheel_radius_error:=0.002 \
    wheel_separation_error:=0.01
```

### 🕹️ Teleoperation

#### Joystick Control
```bash
# Launch joystick teleoperation
ros2 launch simubot_controller joystick_teleop.launch.py

# For simulation
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=true

# For real robot
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=false
```

#### Keyboard Control
```bash
# Manual velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 📖 Learning Examples

### Running Python Examples

```bash
# Basic publisher example
ros2 run simubot_py_examples simple_publisher

# Subscriber example
ros2 run simubot_py_examples simple_subscriber

# Service server
ros2 run simubot_py_examples simple_service_server

# Service client
ros2 run simubot_py_examples simple_service_client

# Parameter handling
ros2 run simubot_py_examples simple_parameter

# Lifecycle node management
ros2 run simubot_py_examples simple_lifecycle_node

# Transform and kinematics
ros2 run simubot_py_examples simple_tf_kinematics
ros2 run simubot_py_examples simple_turtlesim_kinematics
```

### Running C++ Examples

```bash
# Basic publisher example
ros2 run simubot_cpp_examples simple_publisher

# Subscriber example
ros2 run simubot_cpp_examples simple_subscriber

# Service server
ros2 run simubot_cpp_examples simple_service_server

# Service client
ros2 run simubot_cpp_examples simple_service_client

# Parameter handling
ros2 run simubot_cpp_examples simple_parameter

# Lifecycle node management
ros2 run simubot_cpp_examples simple_lifecycle_node

# Transform and kinematics
ros2 run simubot_cpp_examples simple_tf_kinematics
ros2 run simubot_cpp_examples simple_turtlesim_kinematics
```

## 🔧 Development Workflow

### Building Individual Packages

```bash
# Build specific package
colcon build --packages-select simubot_controller

# Build with verbose output
colcon build --event-handlers console_direct+

# Build in debug mode
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing and Verification

```bash
# Run package tests
colcon test --packages-select simubot_controller

# Check test results
colcon test-result --verbose

# Lint and style checking
ament_lint_auto --packages-select simubot_controller
```

### Debugging and Monitoring

```bash
# Monitor robot topics
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /joint_states

# Check node status
ros2 node list
ros2 node info /simple_controller

# Monitor transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link odom

# Parameter inspection
ros2 param list
ros2 param get /simple_controller wheel_radius
```

## 🌐 Network and Multi-Robot Support

### Multi-Machine Setup

```bash
# Set ROS_DOMAIN_ID for isolation
export ROS_DOMAIN_ID=1

# Network discovery
export ROS_DISCOVERY_RANGE=localhost  # Local only
export ROS_DISCOVERY_RANGE=subnet     # Subnet discovery
```

### Robot Fleet Management

```bash
# Launch multiple robots with different namespaces
ros2 launch simubot_bringup simulated_robot.launch.py robot_name:=robot1
ros2 launch simubot_bringup simulated_robot.launch.py robot_name:=robot2
```

## 🛠️ Troubleshooting

### Common Issues and Solutions

#### Build Issues
```bash
# Clean build
rm -rf build install log
colcon build

# Dependency issues
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### Runtime Issues
```bash
# Source workspace in every terminal
source install/setup.bash

# Check for missing dependencies
ldd install/simubot_controller/lib/simubot_controller/simple_controller

# Verify Gazebo installation
gz sim --version  # For newer ROS 2 versions
gazebo --version  # For Humble
```

#### Controller Issues
```bash
# Check joint states
ros2 topic echo /joint_states

# Verify wheel commands
ros2 topic echo /simple_velocity_controller/commands

# Check transform tree
ros2 run tf2_tools view_frames
```

## 📚 Technical Specifications

### Robot Configuration
- **Type**: Differential Drive Robot
- **Wheel Radius**: 33mm (configurable)
- **Wheel Separation**: 170mm (configurable)
- **Maximum Linear Velocity**: 1.0 m/s
- **Maximum Angular Velocity**: 2.0 rad/s

### Sensor Suite
- **IMU**: 6-DOF inertial measurement unit
- **Wheel Encoders**: High-resolution position feedback
- **Optional**: Camera, LiDAR (extensible)

### Control Features
- **Differential Drive Kinematics**: Forward and inverse kinematic models
- **Odometry Estimation**: Dead-reckoning from wheel encoders
- **Sensor Fusion**: EKF-based state estimation
- **Noise Simulation**: Realistic sensor error modeling
- **Real-time Performance**: Low-latency control loops

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/new-feature`
3. Commit changes: `git commit -am 'Add new feature'`
4. Push to branch: `git push origin feature/new-feature`
5. Submit a Pull Request

### Development Guidelines
- Follow ROS 2 coding standards
- Add comprehensive documentation
- Include unit tests for new features
- Ensure cross-platform compatibility

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Maintainer

- **Ayush** - [am2836166@gmail.com](mailto:am2836166@gmail.com)

## 🙏 Acknowledgments

- ROS 2 Community for the excellent framework
- Gazebo team for the physics simulation environment
- Open Source Robotics Foundation for the ecosystem support

## 📈 Roadmap

- [ ] Advanced path planning algorithms
- [ ] SLAM (Simultaneous Localization and Mapping) integration
- [ ] Multi-robot coordination
- [ ] Machine learning-based control
- [ ] Extended sensor support (cameras, LiDAR)
- [ ] Real-time visualization improvements
- [ ] Mobile app interface for remote control

---
