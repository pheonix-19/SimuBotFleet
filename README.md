# SimuBotFleet - ROS 2 Multi-Robot Fleet Management System

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron%20%7C%20Jazzy-blue.svg)](https://docs.ros.org/en/rolling/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/downloads/)
[![Docker](https://img.shields.io/badge/Docker-Supported-blue.svg)](https://www.docker.com/)

## Overview

SimuBotFleet is a comprehensive ROS 2 workspace designed for multi-robot fleet management, simulation, and control. It provides a complete ecosystem for differential drive robot development, including Gazebo simulation, real hardware interfaces, autonomous navigation, computer vision, task allocation, and cloud-based fleet monitoring. The project supports both simulated and real robot operations with extensive examples, microservices architecture, and production-ready deployment tools.

## 🏗️ Project Architecture

```
SimuBotFleet/
├── src/
│   ├── 🤖 ROS 2 Robot Packages
│   │   ├── simubot_bringup/          # Single robot launch configurations
│   │   ├── simubot_controller/       # Robot control and odometry
│   │   ├── simubot_description/      # URDF models and visualization
│   │   ├── simubot_localization/     # Robot localization using EKF
│   │   ├── simubot_msgs/             # Custom message definitions
│   │   ├── simubot_py_examples/      # Python ROS 2 learning examples
│   │   └── simubot_cpp_examples/     # C++ ROS 2 learning examples
│   ├── 🚁 Fleet Management Packages
│   │   ├── simubot_fleet_bringup/    # Multi-robot fleet orchestration
│   │   ├── simubot_task_allocator/   # AI-powered task assignment
│   │   └── simubot_vision/           # Computer vision and detection
│   └── 🌐 Cloud Services
│       └── services/
│           ├── simubot_api/          # REST API and ROS bridge
│           ├── simubot_db/           # Database schema
│           ├── simubot_monitoring/   # Prometheus metrics
│           └── deployment/           # Docker/K8s orchestration
├── build/                            # Build artifacts
├── install/                          # Installation directory
├── log/                              # Build and runtime logs
└── scripts/                          # Utility scripts
```

## 📦 Package Descriptions

### 🤖 Core Robot Packages

#### 🚀 simubot_bringup
**Single robot orchestration and launch management**

- **Purpose**: Provides high-level launch files for individual robot deployment scenarios
- **Key Features**:
  - Simulated robot launch configuration with Gazebo integration
  - Real robot hardware interface launch with sensor fusion
  - Integrated sensor and control system startup
  - Parameter management for different robot configurations

**Usage Commands**:
```bash
# Launch complete simulated robot system
ros2 launch simubot_bringup simulated_robot.launch.py

# Launch real robot hardware interface
ros2 launch simubot_bringup real_robot.launch.py

# Launch with custom parameters
ros2 launch simubot_bringup simulated_robot.launch.py \
    world_name:=warehouse \
    robot_name:=robot_1 \
    use_rviz:=true
```

#### 🎮 simubot_controller
**Advanced robot control, kinematics, and odometry system**

- **Purpose**: Implements differential drive robot control with multiple controller variants
- **Key Features**:
  - **Simple Controller**: Basic differential drive controller for velocity commands
  - **Noisy Controller**: Advanced controller with realistic sensor noise simulation
  - **Dual Implementation**: Both Python and C++ versions for performance comparison
  - **Real-time Odometry**: Dead-reckoning position and velocity estimation
  - **Joystick Teleoperation**: Manual robot control using game controllers
  - **Wheel Kinematics**: Forward and inverse kinematics for differential drive robots

**Controller Parameters**:
- `wheel_radius`: 0.033m (default wheel radius)
- `wheel_separation`: 0.17m (default wheel base)
- `wheel_radius_error`: 0.005m (noise simulation for noisy controller)
- `wheel_separation_error`: 0.02m (geometric error simulation)

**Usage Commands**:
```bash
# Launch controller with default settings
ros2 launch simubot_controller controller.launch.py

# Use simple controller (basic functionality)
ros2 launch simubot_controller controller.launch.py use_simple_controller:=true

# Use noisy controller with sensor simulation
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

# Launch joystick teleoperation
ros2 launch simubot_controller joystick_teleop.launch.py

# Joystick for simulation
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=true

# Joystick for real robot
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=false
```

**Manual Control Commands**:
```bash
# Forward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

#### 🤖 simubot_description
**Robot model definition, visualization, and simulation setup**

- **Purpose**: Contains URDF/Xacro robot models and visualization configurations
- **Key Features**:
  - **URDF Model**: Complete robot description with meshes and physics properties
  - **Gazebo Integration**: Simulation-ready robot model with realistic physics
  - **RViz Configuration**: Pre-configured visualization setup for development
  - **Multi-platform Support**: Compatible with Gazebo Classic and Gazebo Garden/Ignition
  - **Sensor Integration**: IMU, wheel encoders, cameras, and other sensor models
  - **Material Definitions**: Realistic textures and visual properties

**Usage Commands**:
```bash
# Launch robot visualization with GUI controls
ros2 launch simubot_description display.launch.py

# Launch robot model without GUI (headless systems)
ros2 launch simubot_description display_no_gui.launch.py

# Launch Gazebo simulation environment
ros2 launch simubot_description gazebo.launch.py

# Launch with custom world
ros2 launch simubot_description gazebo.launch.py world_name:=warehouse

# Visualize robot in specific configuration
ros2 launch simubot_description display.launch.py \
    robot_name:=robot_1 \
    use_joint_state_publisher_gui:=true
```

#### 📍 simubot_localization
**Advanced robot state estimation and localization**

- **Purpose**: Provides accurate robot pose estimation using multi-sensor fusion
- **Key Features**:
  - **Extended Kalman Filter (EKF)**: Fuses wheel odometry, IMU, and GPS data
  - **IMU Integration**: Processes accelerometer, gyroscope, and magnetometer data
  - **Transform Publishing**: Maintains accurate coordinate frame relationships
  - **Noise Handling**: Robust estimation despite sensor noise and outliers
  - **Multi-sensor Fusion**: Combines multiple data sources for better accuracy

**Usage Commands**:
```bash
# Launch EKF-based localization
ros2 launch simubot_localization local_localization.launch.py

# Use Python implementation
ros2 launch simubot_localization local_localization.launch.py use_python:=true

# Use C++ implementation (default)
ros2 launch simubot_localization local_localization.launch.py use_python:=false

# Custom EKF parameters
ros2 launch simubot_localization local_localization.launch.py \
    base_frame:=base_link \
    odom_frame:=odom \
    world_frame:=map
```

#### 📨 simubot_msgs
**Custom message and service definitions**

- **Purpose**: Defines custom interfaces for robot-specific communication
- **Key Features**:
  - Custom message types for robot fleet communication
  - Service definitions for robot operations and coordination
  - Action interfaces for long-running tasks
  - Standardized communication protocols across the fleet

**Usage Commands**:
```bash
# Build messages package
colcon build --packages-select simubot_msgs

# List available custom messages
ros2 interface list | grep simubot_msgs

# Show message definition
ros2 interface show simubot_msgs/msg/CustomMessage

# Show service definition
ros2 interface show simubot_msgs/srv/CustomService
```

### 🚁 Fleet Management Packages

#### 🚁 simubot_fleet_bringup
**Multi-robot fleet orchestration and coordination**

- **Purpose**: Orchestrates and manages multiple robots as a coordinated fleet
- **Key Features**:
  - **Multi-robot Launch**: Simultaneous deployment of multiple robots
  - **Namespace Management**: Isolated robot instances with unique identifiers
  - **Nav2 Integration**: Fleet-wide navigation and path planning
  - **Coordinated Control**: Synchronized robot operations
  - **Scalable Architecture**: Support for 2+ robots with parameter-driven scaling

**Usage Commands**:
```bash
# Launch multi-robot fleet (default 2 robots)
ros2 launch simubot_fleet_bringup fleet.launch.py

# Launch with custom robot count
ros2 launch simubot_fleet_bringup fleet.launch.py robot_count:=4

# Launch fleet with navigation
ros2 launch simubot_fleet_bringup fleet.launch.py \
    robot_count:=3 \
    use_navigation:=true \
    map_file:=/path/to/warehouse.yaml

# Launch fleet in simulation
ros2 launch simubot_fleet_bringup fleet.launch.py \
    use_simulation:=true \
    world_name:=warehouse
```

#### 🧠 simubot_task_allocator
**AI-powered task assignment and fleet coordination**

- **Purpose**: Intelligent task allocation system for multi-robot coordination
- **Key Features**:
  - **Nearest Robot Policy**: Distance-based task assignment algorithm
  - **Priority Queue Management**: Task prioritization and scheduling
  - **Nav2 Goal Publishing**: Integration with navigation stack
  - **JSON Task Interface**: RESTful task definition and submission
  - **Extensible AI Framework**: Ready for reinforcement learning integration
  - **Real-time Allocation**: Dynamic task redistribution based on robot availability

**Node Configuration**:
- Default robot count: 2 (configurable via parameter)
- Task subscription: `/fleet/tasks` (JSON format)
- Goal publishing: `/robot_N/goal_pose` (Nav2 compatible)

**Usage Commands**:
```bash
# Launch task allocator (default 2 robots)
ros2 launch simubot_task_allocator task_allocator.launch.py

# Launch with custom robot count
ros2 launch simubot_task_allocator task_allocator.launch.py robot_count:=4

# Run task allocator node directly
ros2 run simubot_task_allocator task_allocator_node --ros-args -p robot_count:=3

# Submit task via command line
ros2 topic pub /fleet/tasks std_msgs/msg/String \
    'data: "{\"pick\": [1.0, 2.0], \"drop\": [3.0, 4.0], \"priority\": 1}"'

# Monitor task allocation
ros2 topic echo /robot_1/goal_pose
ros2 topic echo /robot_2/goal_pose
```

**Task JSON Format**:
```json
{
  "pick": [x, y],           # Pick-up coordinates
  "drop": [x, y],           # Drop-off coordinates  
  "priority": 1             # Task priority (1=highest)
}
```

#### 👁️ simubot_vision
**Computer vision and object detection system**

- **Purpose**: Real-time computer vision processing for inventory and object detection
- **Key Features**:
  - **Real-time Image Processing**: OpenCV-based computer vision pipeline
  - **Object Detection**: Package and inventory item recognition
  - **Event Publishing**: JSON-formatted detection events
  - **Camera Integration**: Support for USB, ROS, and IP cameras
  - **Configurable Detection**: Customizable detection algorithms
  - **Multi-robot Support**: Namespace-aware for fleet operations

**Usage Commands**:
```bash
# Launch vision processing
ros2 launch simubot_vision vision.launch.py

# Run vision node directly
ros2 run simubot_vision vision_node

# Launch with custom camera topic
ros2 run simubot_vision vision_node --ros-args \
    --remap /camera/image_raw:=/robot_1/camera/image_raw

# Monitor detection events
ros2 topic echo /inventory/events

# Test with image topic
ros2 topic list | grep image
```

**Detection Event Format**:
```json
{
  "type": "package_detected",
  "count": 3,
  "timestamp": "2025-08-16T10:30:00Z"
}
```

### 📚 Learning and Example Packages

#### 🐍 simubot_py_examples
**Comprehensive Python ROS 2 learning examples**

- **Purpose**: Educational examples for mastering ROS 2 concepts in Python
- **Available Examples**:
  - **simple_publisher.py**: Basic topic publishing patterns
  - **simple_subscriber.py**: Topic subscription and message handling
  - **simple_service_server.py**: Service server implementation
  - **simple_service_client.py**: Service client usage and async calls
  - **simple_parameter.py**: Parameter handling and dynamic reconfiguration
  - **simple_lifecycle_node.py**: Managed node lifecycle implementation
  - **simple_tf_kinematics.py**: Transform operations and coordinate frames
  - **simple_turtlesim_kinematics.py**: Turtle robot kinematics and control

**Usage Commands**:
```bash
# Basic publisher example
ros2 run simubot_py_examples simple_publisher

# Subscriber example
ros2 run simubot_py_examples simple_subscriber

# Service server (run in separate terminal)
ros2 run simubot_py_examples simple_service_server

# Service client (after starting server)
ros2 run simubot_py_examples simple_service_client

# Parameter handling demonstration
ros2 run simubot_py_examples simple_parameter

# Lifecycle node management
ros2 run simubot_py_examples simple_lifecycle_node

# Transform and coordinate frame operations
ros2 run simubot_py_examples simple_tf_kinematics

# Turtle robot kinematics (requires turtlesim)
ros2 run turtlesim turtlesim_node &
ros2 run simubot_py_examples simple_turtlesim_kinematics
```

#### ⚡ simubot_cpp_examples
**High-performance C++ ROS 2 learning examples**

- **Purpose**: Performance-optimized examples for learning ROS 2 concepts in C++
- **Available Examples**:
  - **simple_publisher.cpp**: Efficient topic publishing with minimal latency
  - **simple_subscriber.cpp**: Fast message processing and callbacks
  - **simple_service_server.cpp**: High-performance service implementation
  - **simple_service_client.cpp**: Asynchronous service calls and futures
  - **simple_parameter.cpp**: Parameter management and callbacks
  - **simple_lifecycle_node.cpp**: Managed node lifecycle in C++
  - **simple_tf_kinematics.cpp**: High-performance transform calculations
  - **simple_turtlesim_kinematics.cpp**: Real-time robot kinematics implementation

**Usage Commands**:
```bash
# Basic publisher example
ros2 run simubot_cpp_examples simple_publisher

# Subscriber example
ros2 run simubot_cpp_examples simple_subscriber

# Service server (run in separate terminal)
ros2 run simubot_cpp_examples simple_service_server

# Service client (after starting server)
ros2 run simubot_cpp_examples simple_service_client

# Parameter handling demonstration
ros2 run simubot_cpp_examples simple_parameter

# Lifecycle node management
ros2 run simubot_cpp_examples simple_lifecycle_node

# High-performance transform operations
ros2 run simubot_cpp_examples simple_tf_kinematics

# Turtle robot kinematics (requires turtlesim)
ros2 run turtlesim turtlesim_node &
ros2 run simubot_cpp_examples simple_turtlesim_kinematics
```

### 🌐 Cloud Services and Infrastructure

#### 🚀 simubot_api
**REST API service and ROS-Cloud bridge**

- **Purpose**: FastAPI-based REST service bridging ROS 2 fleet with cloud infrastructure
- **Key Features**:
  - **RESTful Task Submission**: HTTP API for task assignment
  - **ROS Integration**: Seamless bridge between REST API and ROS 2 topics
  - **Queue Management**: Redis and RabbitMQ integration for task queuing
  - **Prometheus Metrics**: Built-in monitoring and metrics collection
  - **Async Processing**: Celery-based background task processing
  - **Database Integration**: PostgreSQL support for persistent data storage

**Technology Stack**:
- **Framework**: FastAPI with async/await support
- **Queue**: RabbitMQ for message brokering
- **Cache**: Redis for session and cache management
- **Database**: PostgreSQL for persistent storage
- **Monitoring**: Prometheus metrics integration
- **Containerization**: Docker-ready with multi-stage builds

**API Endpoints**:
```bash
# Submit task via REST API
curl -X POST http://localhost:8000/assign_task \
  -H "Content-Type: application/json" \
  -d '{
    "pick": [1.5, 2.0],
    "drop": [3.0, 1.5], 
    "priority": 1
  }'

# Check API metrics
curl http://localhost:8000/metrics

# Health check
curl http://localhost:8000/health
```

**Usage Commands**:
```bash
# Build API service
cd src/services/simubot_api
docker build -t simubot-api .

# Run API service locally
docker run -p 8000:8000 simubot-api

# Run with environment variables
docker run -p 8000:8000 \
  -e RABBITMQ_URL=amqp://guest:guest@localhost:5672/ \
  simubot-api

# Check API logs
docker logs simubot-api

# Interactive development
pip install -r requirements.txt
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

#### 🗄️ simubot_db
**Database schema and data management**

- **Purpose**: PostgreSQL database schema for fleet data persistence
- **Key Features**:
  - **Robot Registry**: Robot status, battery, and capability tracking
  - **Task Management**: Task queue, status, and history
  - **Inventory Events**: Computer vision detection event logging
  - **Audit Trail**: Comprehensive logging for debugging and analytics
  - **Scalable Schema**: Designed for multi-robot fleet operations

**Database Schema**:
- **robots**: Robot registration and status tracking
- **tasks**: Task queue and execution history
- **inventory_events**: Vision system detection events

**Usage Commands**:
```bash
# Connect to database (when running via docker-compose)
docker exec -it deployment_postgres_1 psql -U simubot -d simubot

# View robot status
SELECT * FROM robots;

# Check recent tasks
SELECT * FROM tasks ORDER BY created_at DESC LIMIT 10;

# Monitor inventory events
SELECT * FROM inventory_events ORDER BY created_at DESC LIMIT 5;

# Database backup
docker exec deployment_postgres_1 pg_dump -U simubot simubot > backup.sql

# Restore database
docker exec -i deployment_postgres_1 psql -U simubot simubot < backup.sql
```

#### 📊 simubot_monitoring
**Fleet monitoring and observability**

- **Purpose**: Prometheus-based monitoring and metrics collection
- **Key Features**:
  - **Real-time Metrics**: Fleet performance and health monitoring
  - **Custom Dashboards**: Grafana visualization for fleet operations
  - **Alert Management**: Automated alerts for system issues
  - **Historical Data**: Long-term trend analysis and capacity planning
  - **Multi-robot Tracking**: Individual robot and fleet-wide metrics

**Monitoring Stack**:
- **Prometheus**: Metrics collection and time-series storage
- **Grafana**: Visualization and dashboard creation
- **AlertManager**: Alert routing and notification management

**Usage Commands**:
```bash
# Access Prometheus (when running via docker-compose)
open http://localhost:9090

# Access Grafana dashboards
open http://localhost:3000
# Default credentials: admin/admin

# Query fleet metrics
curl 'http://localhost:9090/api/v1/query?query=simubot_tasks_created_total'

# Check monitoring configuration
cat src/services/simubot_monitoring/prometheus.yml
```

#### 🚢 deployment
**Production deployment and orchestration**

- **Purpose**: Docker Compose and Kubernetes deployment configurations
- **Key Features**:
  - **Docker Compose**: Local development and testing deployment
  - **Kubernetes**: Production-ready container orchestration
  - **Service Discovery**: Automatic service registration and discovery
  - **Load Balancing**: Traffic distribution across service instances
  - **Scaling Configuration**: Horizontal pod autoscaling setup
  - **Persistent Storage**: Volume management for stateful services

**Services Included**:
- PostgreSQL database with persistent storage
- RabbitMQ message broker with management interface
- SimuBot API with auto-scaling configuration
- Queue worker processes for background tasks
- Prometheus monitoring stack
- Grafana dashboards and visualization

**Usage Commands**:
```bash
# Start complete stack with Docker Compose
cd src/services/deployment
docker-compose up -d

# View service status
docker-compose ps

# Check service logs
docker-compose logs simubot-api
docker-compose logs postgres

# Scale API service
docker-compose up -d --scale simubot-api=3

# Stop all services
docker-compose down

# Stop and remove volumes (WARNING: destroys data)
docker-compose down -v

# Kubernetes deployment (requires kubectl configured)
cd k8s/
kubectl apply -f .

# Check pod status
kubectl get pods

# Scale deployment
kubectl scale deployment simubot-api --replicas=5

# Port forwarding for local access
kubectl port-forward service/simubot-api 8000:8000
```

**Service URLs (when running locally)**:
- **API Service**: http://localhost:8000
- **API Documentation**: http://localhost:8000/docs
- **RabbitMQ Management**: http://localhost:15672 (guest/guest)
- **Prometheus**: http://localhost:9090
- **Grafana**: http://localhost:3000 (admin/admin)
- **PostgreSQL**: localhost:5432 (simubot/simubot)

## 🚀 Installation and Setup

### Prerequisites

- **ROS 2**: Humble, Iron, or Jazzy distribution
- **Gazebo**: Garden, Ignition, or Classic (depending on ROS 2 version)
- **Python**: 3.8+ with pip
- **Docker**: Latest version (for cloud services)
- **Docker Compose**: v2.0+ (for development deployment)
- **Git**: For version control and repository management

### System Dependencies

#### Ubuntu/Debian
```bash
# Install ROS 2 (example for Jazzy)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Install additional dependencies
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgeographic-dev \
    libpugixml-dev \
    libboost-all-dev

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt install docker-compose-plugin
```

### Workspace Setup

#### 1. Clone and Build ROS 2 Workspace

```bash
# Clone the repository
git clone https://github.com/pheonix-19/SimuBotFleet.git
cd SimuBotFleet

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

#### 2. Cloud Services Setup

```bash
# Navigate to deployment directory
cd src/services/deployment

# Start cloud services (PostgreSQL, RabbitMQ, API, Monitoring)
docker-compose up -d

# Wait for services to initialize (about 30 seconds)
docker-compose ps

# Verify services are running
curl http://localhost:8000/metrics
curl http://localhost:9090/targets
```

#### 3. Verification and Testing

```bash
# Test ROS 2 installation
ros2 topic list
ros2 node list

# Test robot simulation
ros2 launch simubot_bringup simulated_robot.launch.py

# In another terminal, test API integration
curl -X POST http://localhost:8000/assign_task \
  -H "Content-Type: application/json" \
  -d '{"pick": [1.0, 2.0], "drop": [3.0, 4.0], "priority": 1}'

# Test fleet coordination
ros2 launch simubot_fleet_bringup fleet.launch.py robot_count:=2
```

## 🎮 Usage and Operation

### 🤖 Single Robot Operations

#### Simulated Robot Deployment
```bash
# Launch complete simulated robot with Gazebo, controller, and joystick
ros2 launch simubot_bringup simulated_robot.launch.py

# Launch with specific configuration
ros2 launch simubot_bringup simulated_robot.launch.py \
    world_name:=warehouse \
    robot_name:=simubot_01 \
    use_rviz:=true \
    use_joystick:=true

# Individual component launches
ros2 launch simubot_description gazebo.launch.py world_name:=warehouse
ros2 launch simubot_controller controller.launch.py use_simple_controller:=false use_python:=false
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=true
ros2 launch simubot_localization local_localization.launch.py
```

#### Real Robot Deployment
```bash
# Launch complete real robot system
ros2 launch simubot_bringup real_robot.launch.py

# Launch with hardware configuration
ros2 launch simubot_bringup real_robot.launch.py \
    robot_name:=simubot_hardware_01 \
    use_localization:=true \
    use_navigation:=true

# Individual component launches
ros2 launch simubot_firmware hardware_interface.launch.py  # (requires simubot_firmware package)
ros2 launch simubot_controller controller.launch.py \
    use_simple_controller:=false \
    use_python:=false \
    use_sim_time:=false
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=false
ros2 launch simubot_localization local_localization.launch.py use_sim_time:=false
```

### 🚁 Multi-Robot Fleet Operations

#### Fleet Launch and Coordination
```bash
# Launch 2-robot fleet (default configuration)
ros2 launch simubot_fleet_bringup fleet.launch.py

# Launch large fleet with navigation
ros2 launch simubot_fleet_bringup fleet.launch.py \
    robot_count:=5 \
    use_navigation:=true \
    world_name:=warehouse \
    map_file:=/path/to/warehouse.yaml

# Fleet with task allocation system
ros2 launch simubot_fleet_bringup fleet.launch.py \
    robot_count:=3 \
    use_task_allocator:=true \
    use_vision:=true

# Mixed simulation and real robots (advanced)
ros2 launch simubot_fleet_bringup fleet.launch.py \
    robot_count:=4 \
    sim_robots:=2 \
    real_robots:=2
```

#### Task Management and Assignment
```bash
# Launch task allocation system
ros2 launch simubot_task_allocator task_allocator.launch.py robot_count:=3

# Submit tasks via ROS topics
ros2 topic pub /fleet/tasks std_msgs/msg/String \
    'data: "{\"pick\": [2.0, 3.0], \"drop\": [5.0, 1.0], \"priority\": 1}"'

# Submit multiple tasks with different priorities
ros2 topic pub /fleet/tasks std_msgs/msg/String \
    'data: "{\"pick\": [0.0, 0.0], \"drop\": [10.0, 10.0], \"priority\": 2}"'

# Submit tasks via REST API (requires cloud services)
curl -X POST http://localhost:8000/assign_task \
  -H "Content-Type: application/json" \
  -d '{"pick": [1.5, 2.5], "drop": [4.0, 3.0], "priority": 1}'

# Monitor task assignments
ros2 topic echo /robot_1/goal_pose
ros2 topic echo /robot_2/goal_pose
ros2 topic echo /robot_3/goal_pose
```

### 👁️ Computer Vision and Detection

#### Vision Processing
```bash
# Launch vision system for single robot
ros2 launch simubot_vision vision.launch.py

# Launch vision for specific robot in fleet
ros2 run simubot_vision vision_node --ros-args \
    --remap /camera/image_raw:=/robot_2/camera/image_raw \
    --remap /inventory/events:=/robot_2/inventory/events

# Monitor detection events
ros2 topic echo /inventory/events

# Test with image publisher
ros2 run image_publisher image_publisher /path/to/test/image.jpg
```

### 🌐 Cloud Services Integration

#### API Service Operations
```bash
# Start cloud services stack
cd src/services/deployment
docker-compose up -d

# Submit task via REST API
curl -X POST http://localhost:8000/assign_task \
  -H "Content-Type: application/json" \
  -d '{
    "pick": [2.0, 1.5],
    "drop": [5.0, 3.0],
    "priority": 1
  }'

# Check API health and metrics
curl http://localhost:8000/health
curl http://localhost:8000/metrics

# View API documentation
open http://localhost:8000/docs
```

#### Database Operations
```bash
# Connect to fleet database
docker exec -it deployment_postgres_1 psql -U simubot -d simubot

# Check robot fleet status
SELECT name, battery, status FROM robots;

# View recent tasks
SELECT pick_x, pick_y, drop_x, drop_y, status, created_at 
FROM tasks 
ORDER BY created_at DESC 
LIMIT 10;

# Monitor inventory events
SELECT event, created_at 
FROM inventory_events 
ORDER BY created_at DESC 
LIMIT 5;
```

#### Monitoring and Observability
```bash
# Access Prometheus metrics (web interface)
open http://localhost:9090

# Access Grafana dashboards
open http://localhost:3000
# Login: admin/admin

# Query specific metrics via CLI
curl 'http://localhost:9090/api/v1/query?query=simubot_tasks_created_total'
curl 'http://localhost:9090/api/v1/query?query=up{job="simubot_api"}'

# Check RabbitMQ queue status
open http://localhost:15672
# Login: guest/guest
```

### 📊 Visualization and Monitoring

#### RViz Robot Visualization
```bash
# Launch robot model visualization with GUI controls
ros2 launch simubot_description display.launch.py

# Multi-robot visualization
ros2 launch simubot_description display.launch.py \
    robot_name:=robot_1 \
    namespace:=/robot_1

# Launch without GUI (headless systems)
ros2 launch simubot_description display_no_gui.launch.py

# Custom RViz configuration
ros2 launch simubot_description display.launch.py \
    rviz_config:=/path/to/custom.rviz
```

#### Real-time System Monitoring
```bash
# Monitor robot topics
ros2 topic list
ros2 topic echo /robot_1/odom
ros2 topic echo /robot_2/joint_states
ros2 topic hz /cmd_vel

# Check node status
ros2 node list
ros2 node info /simple_controller
ros2 lifecycle list /managed_node

# Monitor transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link odom
ros2 run tf2_ros tf2_echo map base_link

# Parameter inspection
ros2 param list
ros2 param get /simple_controller wheel_radius
ros2 param set /simple_controller wheel_radius 0.035
```

### 🕹️ Teleoperation and Manual Control

#### Joystick Control
```bash
# Launch joystick teleoperation for simulation
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=true

# Launch joystick for real robot
ros2 launch simubot_controller joystick_teleop.launch.py use_sim_time:=false

# Launch joystick for specific robot in fleet
ros2 launch simubot_controller joystick_teleop.launch.py \
    robot_name:=robot_2 \
    use_sim_time:=true

# Test joystick connectivity
ros2 topic echo /joy
ls /dev/input/js*
```

#### Keyboard Control Commands
```bash
# Forward movement
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Backward movement
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Rotate left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Rotate right
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'

# Combined movement (arc)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'

# Emergency stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Control specific robot in fleet
ros2 topic pub --once /robot_1/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 🔧 Development Workflow

### Building and Package Management

#### Building Individual Packages
```bash
# Build specific package
colcon build --packages-select simubot_controller

# Build with dependencies
colcon build --packages-up-to simubot_task_allocator

# Build with verbose output for debugging
colcon build --event-handlers console_direct+

# Build in debug mode
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build with parallel jobs
colcon build --parallel-workers 4

# Clean build (remove build/install directories)
rm -rf build install log
colcon build

# Build only Python packages
colcon build --packages-select simubot_py_examples simubot_task_allocator simubot_vision

# Build only C++ packages
colcon build --packages-select simubot_cpp_examples simubot_controller
```

#### Package Development
```bash
# Create new ROS 2 package (Python)
ros2 pkg create --build-type ament_python my_new_package --dependencies rclpy

# Create new ROS 2 package (C++)
ros2 pkg create --build-type ament_cmake my_new_package --dependencies rclcpp

# Add package to workspace
cd src/
# ... develop your package ...
cd ..
colcon build --packages-select my_new_package

# Source and test
source install/setup.bash
ros2 run my_new_package my_node
```

### Testing and Quality Assurance

#### Running Tests
```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select simubot_controller

# Run tests with verbose output
colcon test --event-handlers console_direct+

# Check test results
colcon test-result --verbose

# Run tests and show failures only
colcon test-result --all | grep -E "(FAIL|ERROR)"

# Continuous testing during development
while inotifywait -e modify src/; do colcon test --packages-select simubot_controller; done
```

#### Code Quality and Linting
```bash
# Lint Python code
ament_flake8 src/simubot_py_examples/
ament_pep257 src/simubot_py_examples/

# Lint C++ code
ament_cppcheck src/simubot_cpp_examples/
ament_cpplint src/simubot_cpp_examples/

# Copyright check
ament_copyright src/

# Run all linting tools
colcon test --packages-select simubot_py_examples --pytest-args -v
```

#### Performance Testing
```bash
# Profile node performance
ros2 run simubot_controller simple_controller &
PID=$!
perf record -p $PID sleep 10
perf report

# Memory usage analysis
valgrind --tool=memcheck ros2 run simubot_cpp_examples simple_publisher

# Network latency testing
ros2 topic hz /odom
ros2 topic bw /cmd_vel
```

### Debugging and Troubleshooting

#### System Diagnostics
```bash
# Check ROS 2 environment
printenv | grep ROS
ros2 doctor

# Verify node connectivity
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

# Check message flow
ros2 topic echo /odom --no-arr
ros2 topic hz /joint_states

# Debug transform tree
ros2 run tf2_tools view_frames
evince frames.pdf
ros2 run tf2_ros tf2_echo base_link odom

# Monitor system resources
htop
ros2 node info /simple_controller
```

#### Log Analysis
```bash
# View ROS 2 logs
ros2 bag record -a  # Record all topics
ros2 bag play <bag_file>

# Set log levels
ros2 run simubot_controller simple_controller --ros-args --log-level DEBUG

# View node logs
ros2 launch simubot_bringup simulated_robot.launch.py > robot.log 2>&1

# Filter logs by severity
grep "ERROR\|WARN" robot.log
```

#### Common Issues and Solutions

##### Build Issues
```bash
# Dependency issues
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean build artifacts
rm -rf build install log
colcon build

# Check for missing dependencies
ldd install/simubot_controller/lib/simubot_controller/simple_controller

# Resolve package conflicts
colcon list --packages-up-to simubot_controller
```

##### Runtime Issues
```bash
# Source workspace in every terminal
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc

# Check topic connections
ros2 topic info /cmd_vel
ros2 node info /teleop_twist_joy

# Verify parameter loading
ros2 param list
ros2 param describe /simple_controller wheel_radius
```

##### Gazebo Issues
```bash
# Check Gazebo version compatibility
gazebo --version  # For Humble
gz sim --version  # For Iron/Jazzy

# Reset Gazebo models
rm -rf ~/.gazebo/models/
rm -rf ~/.ignition/

# GPU acceleration issues
export GAZEBO_MODEL_PATH=/opt/ros/$ROS_DISTRO/share/gazebo_plugins/models
export GAZEBO_PLUGIN_PATH=/opt/ros/$ROS_DISTRO/lib/gazebo_plugins

# Headless Gazebo (no GUI)
export GAZEBO_HEADLESS=1
```

##### Controller Issues
```bash
# Check joint states publication
ros2 topic echo /joint_states

# Verify velocity commands
ros2 topic echo /simple_velocity_controller/commands

# Monitor control loop timing
ros2 topic hz /odom
ros2 topic hz /simple_velocity_controller/commands

# Debug controller parameters
ros2 param get /simple_controller wheel_radius
ros2 param get /simple_controller wheel_separation
```

### Cloud Services Development

#### Local Development Setup
```bash
# Set up Python virtual environment
python3 -m venv venv
source venv/bin/activate
pip install -r src/services/simubot_api/requirements.txt

# Run API in development mode
cd src/services/simubot_api
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Run background worker
python -m app.worker

# Test API endpoints
curl http://localhost:8000/docs
```

#### Database Development
```bash
# Run PostgreSQL locally
docker run --name postgres-dev \
  -e POSTGRES_PASSWORD=dev \
  -e POSTGRES_DB=simubot_dev \
  -p 5432:5432 \
  -d postgres:16

# Connect to development database
psql -h localhost -U postgres -d simubot_dev

# Run migrations
docker exec -i postgres-dev psql -U postgres -d simubot_dev < src/services/simubot_db/schema.sql
```

#### Container Development
```bash
# Build development image
cd src/services/simubot_api
docker build -t simubot-api:dev .

# Run with volume mounting for development
docker run -p 8000:8000 \
  -v $(pwd)/app:/app/app \
  simubot-api:dev

# Debug container issues
docker logs simubot-api
docker exec -it simubot-api bash
```

### Network and Multi-Machine Setup

#### Multi-Machine Configuration
```bash
# Set up ROS_DOMAIN_ID for isolation
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Configure network discovery
export ROS_DISCOVERY_RANGE=localhost    # Local only
export ROS_DISCOVERY_RANGE=subnet       # Subnet discovery
export ROS_DISCOVERY_RANGE=system_default  # System default

# Test network connectivity
ros2 multicast receive
ros2 multicast send

# Check firewall settings (Ubuntu)
sudo ufw status
sudo ufw allow 7400/udp  # ROS 2 discovery
sudo ufw allow 7401/udp  # ROS 2 user traffic
```

#### Fleet Coordination Development
```bash
# Launch multiple robots with unique namespaces
ros2 launch simubot_fleet_bringup fleet.launch.py robot_count:=3

# Test task allocation system
ros2 run simubot_task_allocator task_allocator_node --ros-args -p robot_count:=3

# Monitor fleet coordination
ros2 topic echo /fleet/tasks
ros2 topic echo /robot_1/goal_pose
ros2 topic echo /robot_2/goal_pose

# Debug namespace issues
ros2 topic list | grep robot_
ros2 node list | grep robot_
```

## 📚 Technical Specifications

### Robot Hardware Configuration
- **Robot Type**: Differential Drive Mobile Robot
- **Wheel Radius**: 33mm (default, configurable)
- **Wheel Separation**: 170mm (default, configurable)
- **Maximum Linear Velocity**: 1.0 m/s
- **Maximum Angular Velocity**: 2.0 rad/s
- **Control Frequency**: 50 Hz (20ms control loop)
- **Odometry Publishing Rate**: 30 Hz

### Sensor Suite and Capabilities
- **IMU**: 6-DOF inertial measurement unit (accelerometer + gyroscope)
- **Wheel Encoders**: High-resolution quadrature encoders
- **Camera**: RGB camera for computer vision (optional)
- **LiDAR**: 2D laser scanner for navigation (extensible)
- **GPS**: Global positioning for outdoor operations (extensible)
- **Ultrasonic Sensors**: Obstacle detection (extensible)

### Software Architecture
- **Framework**: ROS 2 (Humble/Iron/Jazzy)
- **Simulation**: Gazebo Classic/Garden
- **Programming Languages**: C++17, Python 3.8+
- **Build System**: Colcon with CMake/setuptools
- **Communication**: DDS (Fast-DDS/Cyclone DDS)
- **Visualization**: RViz2, Gazebo GUI
- **Navigation**: Nav2 stack integration
- **Localization**: robot_localization EKF

### Cloud Infrastructure
- **API Framework**: FastAPI with async/await
- **Database**: PostgreSQL 16 with JSONB support
- **Message Queue**: RabbitMQ with management plugin
- **Cache/Session Store**: Redis
- **Monitoring**: Prometheus + Grafana
- **Containerization**: Docker with multi-stage builds
- **Orchestration**: Docker Compose, Kubernetes support

### Performance Characteristics
- **Control Loop Latency**: < 5ms (C++ implementation)
- **Network Latency**: < 10ms (local network)
- **Memory Usage**: ~50MB per robot node
- **CPU Usage**: < 10% per robot (Intel i5 equivalent)
- **Localization Accuracy**: ±5cm (with good sensor fusion)
- **Navigation Precision**: ±10cm (typical indoor environments)

### Differential Drive Kinematics

#### Forward Kinematics
```
Linear velocity:  v = (v_right + v_left) / 2
Angular velocity: ω = (v_right - v_left) / wheel_separation
```

#### Inverse Kinematics
```
v_right = v + (ω * wheel_separation) / 2
v_left  = v - (ω * wheel_separation) / 2
```

#### Odometry Integration
```
x_new = x_old + v * cos(θ) * dt
y_new = y_old + v * sin(θ) * dt
θ_new = θ_old + ω * dt
```

### Network and Communication

#### ROS 2 Topics
| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint positions |
| `/imu` | `sensor_msgs/Imu` | IMU sensor data |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera image stream |
| `/fleet/tasks` | `std_msgs/String` | Fleet task assignments |
| `/inventory/events` | `std_msgs/String` | Vision detection events |

#### ROS 2 Services
| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/configure_robot` | `simubot_msgs/ConfigureRobot` | Robot configuration |
| `/get_robot_status` | `simubot_msgs/GetRobotStatus` | Robot health check |
| `/emergency_stop` | `std_srvs/Trigger` | Emergency stop service |

#### Transform Frames
- **map**: World coordinate frame
- **odom**: Odometry coordinate frame
- **base_link**: Robot base coordinate frame
- **base_footprint**: Robot ground projection
- **wheel_left_link**: Left wheel frame
- **wheel_right_link**: Right wheel frame
- **imu_link**: IMU sensor frame
- **camera_link**: Camera sensor frame

### Database Schema

#### Robots Table
```sql
CREATE TABLE robots (
  id SERIAL PRIMARY KEY,
  name TEXT UNIQUE NOT NULL,
  battery NUMERIC DEFAULT 100,
  status TEXT DEFAULT 'idle',
  position_x NUMERIC DEFAULT 0,
  position_y NUMERIC DEFAULT 0,
  last_seen TIMESTAMP DEFAULT NOW()
);
```

#### Tasks Table
```sql
CREATE TABLE tasks (
  id SERIAL PRIMARY KEY,
  pick_x NUMERIC NOT NULL,
  pick_y NUMERIC NOT NULL,
  drop_x NUMERIC NOT NULL,
  drop_y NUMERIC NOT NULL,
  priority INT DEFAULT 1,
  status TEXT DEFAULT 'queued',
  assigned_robot TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  completed_at TIMESTAMP
);
```

#### Inventory Events Table
```sql
CREATE TABLE inventory_events (
  id SERIAL PRIMARY KEY,
  robot_name TEXT,
  event JSONB,
  location_x NUMERIC,
  location_y NUMERIC,
  created_at TIMESTAMP DEFAULT NOW()
);
```

### API Specifications

#### REST API Endpoints

##### POST /assign_task
Submit a new task to the fleet
```json
Request:
{
  "pick": [x, y],
  "drop": [x, y],
  "priority": 1
}

Response:
{
  "status": "queued",
  "task_id": 123
}
```

##### GET /robots
List all robots and their status
```json
Response:
[
  {
    "name": "robot_1",
    "battery": 85,
    "status": "busy",
    "position": [2.5, 1.3],
    "last_seen": "2025-08-16T10:30:00Z"
  }
]
```

##### GET /tasks
List all tasks with filtering options
```json
Response:
[
  {
    "id": 123,
    "pick": [1.0, 2.0],
    "drop": [3.0, 4.0],
    "status": "completed",
    "assigned_robot": "robot_1",
    "created_at": "2025-08-16T10:00:00Z"
  }
]
```

##### GET /metrics
Prometheus metrics endpoint
```
# HELP simubot_tasks_created_total Tasks created via REST
# TYPE simubot_tasks_created_total counter
simubot_tasks_created_total 42
```

### Security and Authentication

#### Network Security
- **ROS_DOMAIN_ID**: Logical network isolation
- **Firewall Rules**: Port restrictions for ROS 2 communication
- **VPN Support**: Secure multi-site deployments
- **TLS Encryption**: Secure API communication (production)

#### Access Control
- **API Keys**: REST API authentication (configurable)
- **Database Credentials**: Encrypted password storage
- **Container Isolation**: Docker security best practices
- **Role-based Access**: User permission management (extensible)

### Scalability and Performance

#### Horizontal Scaling
- **Multi-robot Support**: 2-50+ robots per fleet
- **Database Sharding**: Distributed data storage
- **Load Balancing**: API service scaling
- **Container Orchestration**: Kubernetes deployment

#### Vertical Scaling
- **Multi-core Support**: Parallel processing capabilities
- **Memory Optimization**: Efficient memory usage patterns
- **CPU Optimization**: Real-time scheduling priorities
- **GPU Acceleration**: Computer vision processing (optional)

### Deployment Architectures

#### Development Environment
```
Single Machine:
- ROS 2 workspace
- Gazebo simulation
- Local database
- Development API server
```

#### Production Environment
```
Multi-Machine Fleet:
- Robot hardware nodes
- Central coordination server
- Cloud database cluster
- Load-balanced API services
- Monitoring infrastructure
```

#### Hybrid Environment
```
Mixed Deployment:
- Real robots on-site
- Cloud simulation environment
- Distributed task coordination
- Remote monitoring capabilities
```

## 🤝 Contributing

We welcome contributions from the robotics and open-source community! Here's how you can contribute to SimuBotFleet:

### 🚀 Getting Started

1. **Fork the Repository**
   ```bash
   # Fork on GitHub, then clone your fork
   git clone https://github.com/YOUR_USERNAME/SimuBotFleet.git
   cd SimuBotFleet
   git remote add upstream https://github.com/pheonix-19/SimuBotFleet.git
   ```

2. **Set Up Development Environment**
   ```bash
   # Install dependencies
   rosdep install --from-paths src --ignore-src -r -y
   
   # Build workspace
   colcon build
   source install/setup.bash
   
   # Set up cloud services for testing
   cd src/services/deployment
   docker-compose up -d
   ```

3. **Create Feature Branch**
   ```bash
   git checkout -b feature/your-feature-name
   # or
   git checkout -b fix/bug-description
   ```

### 📋 Development Guidelines

#### Code Standards
- **ROS 2 Compliance**: Follow [ROS 2 coding standards](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
- **Python**: Follow PEP 8 style guide
- **C++**: Follow Google C++ Style Guide
- **Documentation**: Use clear docstrings and comments
- **Naming**: Use descriptive variable and function names

#### Package Structure
```bash
# When creating new packages
ros2 pkg create --build-type ament_python my_package --dependencies rclpy
# or
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp

# Follow existing package structure:
my_package/
├── package.xml          # Package metadata
├── setup.py            # Python package setup
├── setup.cfg           # Configuration
├── my_package/
│   ├── __init__.py
│   ├── my_node.py      # Main functionality
│   └── launch/         # Launch files
├── test/               # Unit tests
└── README.md          # Package documentation
```

#### Testing Requirements
```bash
# All new code must include tests
colcon test --packages-select your_package

# Lint checking
ament_flake8 src/your_package/
ament_pep257 src/your_package/

# Integration testing
ros2 launch your_package test_launch.py
```

### 🛠️ Contribution Types

#### 🐛 Bug Fixes
- Report bugs using GitHub Issues
- Include steps to reproduce
- Provide system information (ROS version, OS, etc.)
- Submit minimal test cases

#### ✨ New Features
- Discuss feature ideas in GitHub Discussions
- Create feature proposal with technical details
- Implement with backward compatibility
- Include comprehensive tests and documentation

#### 📚 Documentation
- Improve README files
- Add code comments and docstrings
- Create tutorials and examples
- Update API documentation

#### 🔧 Infrastructure
- Docker/Kubernetes improvements
- CI/CD pipeline enhancements
- Performance optimizations
- Security improvements

### 📝 Pull Request Process

1. **Before Submitting**
   ```bash
   # Ensure tests pass
   colcon test
   
   # Run linting
   colcon test --packages-select your_package --pytest-args -v
   
   # Update documentation
   # Add entry to CHANGELOG.md if applicable
   ```

2. **Submit Pull Request**
   - Use descriptive title and description
   - Reference related issues
   - Include test results
   - Add screenshots/videos for UI changes

3. **Review Process**
   - Maintainers will review within 48 hours
   - Address feedback promptly
   - Keep discussions constructive
   - Be patient during the review process

### 🎯 Areas for Contribution

#### High Priority
- **Multi-robot Navigation**: Advanced path planning algorithms
- **SLAM Integration**: Simultaneous localization and mapping
- **Machine Learning**: RL-based task allocation improvements
- **Sensor Integration**: Camera and LiDAR support
- **Performance**: Real-time optimization

#### Medium Priority
- **Mobile Interface**: Web/mobile app for fleet control
- **Advanced Vision**: Object recognition and tracking
- **Simulation Environments**: More realistic world models
- **Documentation**: Video tutorials and guides
- **Testing**: Automated integration tests

#### Beginner Friendly
- **Example Scripts**: More learning examples
- **Bug Fixes**: Simple issues with clear scope
- **Documentation**: README improvements
- **Configuration**: Parameter optimization
- **Logging**: Better error messages and debugging

### 🏷️ Issue Labels

We use the following labels to categorize issues:

- `bug`: Something isn't working correctly
- `enhancement`: New feature request
- `documentation`: Documentation improvements
- `good first issue`: Good for newcomers
- `help wanted`: Extra attention needed
- `question`: Further information requested
- `robot-control`: Related to robot control systems
- `fleet-management`: Multi-robot coordination
- `cloud-services`: API and infrastructure
- `simulation`: Gazebo and visualization
- `testing`: Test improvements

### 🧪 Testing Guidelines

#### Unit Tests
```python
# Example Python unit test
import unittest
from your_package.your_module import YourClass

class TestYourClass(unittest.TestCase):
    def setUp(self):
        self.instance = YourClass()
    
    def test_basic_functionality(self):
        result = self.instance.method()
        self.assertEqual(result, expected_value)
```

#### Integration Tests
```bash
# Test launch file
ros2 launch your_package integration_test.launch.py

# Test with real/simulated robot
ros2 launch your_package robot_test.launch.py
```

#### Performance Tests
```bash
# Latency testing
ros2 topic hz /your_topic

# Memory profiling
valgrind --tool=memcheck ros2 run your_package your_node

# CPU profiling
perf record ros2 run your_package your_node
```

### 📖 Documentation Standards

#### Code Documentation
```python
def calculate_velocity(wheel_left: float, wheel_right: float) -> Tuple[float, float]:
    """
    Calculate linear and angular velocity from wheel speeds.
    
    Args:
        wheel_left: Left wheel velocity in rad/s
        wheel_right: Right wheel velocity in rad/s
        
    Returns:
        Tuple of (linear_velocity, angular_velocity) in m/s and rad/s
        
    Raises:
        ValueError: If wheel velocities are invalid
    """
    pass
```

#### Launch File Documentation
```python
def generate_launch_description():
    """
    Launch file for robot simulation with configurable parameters.
    
    Launch Arguments:
        robot_name: Name of the robot (default: 'simubot')
        use_simulation: Whether to use Gazebo simulation (default: True)
        world_name: Gazebo world to load (default: 'empty_world')
    """
    pass
```

### 🚨 Code Review Checklist

Before submitting, ensure your code meets these criteria:

#### Functionality
- [ ] Code compiles without warnings
- [ ] All tests pass
- [ ] Feature works as described
- [ ] No breaking changes without discussion

#### Code Quality
- [ ] Follows coding standards
- [ ] No code duplication
- [ ] Error handling implemented
- [ ] Resource cleanup (memory, file handles)

#### Documentation
- [ ] Public functions documented
- [ ] README updated if needed
- [ ] Launch file parameters documented
- [ ] Example usage provided

#### Testing
- [ ] Unit tests for new functions
- [ ] Integration tests for new features
- [ ] Performance impact considered
- [ ] Edge cases handled

### 🎉 Recognition

Contributors will be recognized in:
- **README.md** contributors section
- **Release notes** for major contributions
- **GitHub contributors** page
- **Special mentions** in project announcements

### 📞 Getting Help

- **Discussions**: Use GitHub Discussions for questions
- **Issues**: Report bugs and request features
- **Email**: Contact maintainer at am2836166@gmail.com
- **Documentation**: Check existing docs and examples first

### 📋 Development Roadmap

See our [Project Roadmap](https://github.com/pheonix-19/SimuBotFleet/projects) for:
- Planned features and improvements
- Current development priorities
- Long-term project goals
- Community contribution opportunities

Thank you for contributing to SimuBotFleet! 🤖✨

## 📄 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

### License Summary

```
Copyright 2025 Ayush

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

### Third-Party Licenses

This project uses several open-source libraries and frameworks:

- **ROS 2**: Apache License 2.0
- **Gazebo**: Apache License 2.0
- **FastAPI**: MIT License
- **PostgreSQL**: PostgreSQL License
- **RabbitMQ**: Mozilla Public License 2.0
- **Prometheus**: Apache License 2.0
- **Grafana**: Apache License 2.0
- **Docker**: Apache License 2.0

## 👨‍💻 Maintainer and Contact

### Primary Maintainer
- **Name**: Ayush
- **Email**: [am2836166@gmail.com](mailto:am2836166@gmail.com)
- **GitHub**: [@pheonix-19](https://github.com/pheonix-19)
- **LinkedIn**: [Connect on LinkedIn](https://linkedin.com/in/your-profile)

### Project Information
- **Repository**: [https://github.com/pheonix-19/SimuBotFleet](https://github.com/pheonix-19/SimuBotFleet)
- **Issues**: [Report bugs and request features](https://github.com/pheonix-19/SimuBotFleet/issues)
- **Discussions**: [Community discussions](https://github.com/pheonix-19/SimuBotFleet/discussions)
- **Wiki**: [Project documentation](https://github.com/pheonix-19/SimuBotFleet/wiki)

### Getting Support

#### For Technical Issues
1. **Check Documentation**: Review this README and package-specific docs
2. **Search Issues**: Look for existing solutions in GitHub Issues
3. **Create New Issue**: If not found, create a detailed bug report
4. **Community Support**: Ask questions in GitHub Discussions

#### For Contributions
1. **Read Contributing Guide**: See the Contributing section above
2. **Join Discussions**: Participate in feature planning discussions
3. **Submit Pull Requests**: Follow the PR process for code contributions
4. **Contact Maintainer**: For major contributions or questions

#### For Commercial Support
For commercial licensing, enterprise support, or custom development:
- **Email**: [am2836166@gmail.com](mailto:am2836166@gmail.com)
- **Subject Line**: "SimuBotFleet Commercial Support"

## 🙏 Acknowledgments

### Open Source Community
- **ROS 2 Community** - For providing the excellent robotics framework
- **Gazebo Team** - For the powerful physics simulation environment
- **Open Source Robotics Foundation** - For ecosystem support and resources
- **Nav2 Contributors** - For the autonomous navigation stack
- **FastAPI Team** - For the modern web framework

### Academic and Research
- **Robotics Research Community** - For algorithms and best practices
- **Computer Vision Community** - For object detection and tracking methods
- **Control Systems Research** - For advanced control algorithms
- **Multi-Robot Systems Research** - For coordination and task allocation insights

### Technology Partners
- **Docker Community** - For containerization technology
- **PostgreSQL Global Development Group** - For the robust database system
- **Prometheus Community** - For monitoring and observability tools
- **RabbitMQ Team** - For message queuing infrastructure

### Individual Contributors

Special thanks to all contributors who have helped improve SimuBotFleet:

<!-- This section will be automatically updated -->
- **Ayush** ([@pheonix-19](https://github.com/pheonix-19)) - Project creator and lead maintainer
- **Community Contributors** - All GitHub contributors who have submitted PRs, reported issues, and provided feedback

*To see all contributors, visit the [Contributors page](https://github.com/pheonix-19/SimuBotFleet/graphs/contributors)*

### Tools and Services
- **GitHub** - For code hosting and collaboration platform
- **GitHub Actions** - For continuous integration and deployment
- **Docker Hub** - For container image hosting
- **Shields.io** - For README badges and status indicators

### Inspiration and Learning
This project was inspired by and learned from:
- **TurtleBot** projects and ecosystem
- **ROS 2 Navigation** tutorials and examples
- **Industrial robotics** applications and requirements
- **Cloud robotics** research and implementations
- **Open source** robotics projects and best practices

## 📈 Project Roadmap and Future Plans

### 🎯 Short-term Goals (Next 3 months)
- [ ] **Enhanced Navigation**: Integration with Nav2 advanced features
- [ ] **SLAM Capability**: Add simultaneous localization and mapping
- [ ] **Performance Optimization**: Reduce latency and improve real-time performance
- [ ] **Documentation**: Complete video tutorials and guides
- [ ] **Testing**: Comprehensive automated testing suite

### 🚀 Medium-term Goals (3-12 months)
- [ ] **Machine Learning Integration**: RL-based task allocation and path planning
- [ ] **Advanced Sensor Support**: Camera, LiDAR, and RGB-D integration
- [ ] **Mobile Interface**: Web and mobile app for fleet monitoring
- [ ] **Edge Computing**: Distributed processing capabilities
- [ ] **Security Framework**: Authentication and authorization system

### 🌟 Long-term Vision (1+ years)
- [ ] **Autonomous Warehouse**: Complete warehouse automation solution
- [ ] **Multi-Site Coordination**: Cross-location fleet management
- [ ] **AI-Powered Operations**: Intelligent decision making and optimization
- [ ] **Digital Twin**: Real-time simulation and predictive modeling
- [ ] **Ecosystem Integration**: Third-party plugin architecture

### 🤝 Community Involvement
- **Workshops**: ROS 2 and robotics community workshops
- **Conferences**: Presentations at robotics conferences
- **Education**: University and research collaboration
- **Industry**: Enterprise adoption and case studies

### 📊 Success Metrics
- **Community Growth**: Contributors, stars, and forks
- **Code Quality**: Test coverage, documentation completeness
- **Performance**: Benchmark improvements and optimization
- **Adoption**: Real-world deployments and use cases

---

**Built with ❤️ by the robotics community for the robotics community**

*Last updated: August 16, 2025*
