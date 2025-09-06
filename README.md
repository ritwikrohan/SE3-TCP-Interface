# SE3 TCP Interface - Cartesian Motion Control for UR5e

## Overview

This ROS2 package implements a sophisticated **Cartesian motion controller** for the UR5e robot arm that uses an external SE(3) sensor system communicating over TCP/IP. The system enables real-time visual servoing by computing velocity commands based on the pose difference between a tool and target, both tracked by an external sensor.

## Architecture

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                     SE3 Sensor System                        │
│  ┌──────────────┐        TCP/IP         ┌──────────────┐   │
│  │ TCP Server   │◄──────────────────────►│ Hardware     │   │
│  │ (Sensor Sim) │       Port 12345       │ Interface    │   │
│  └──────────────┘                        └──────────────┘   │
└─────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    Cartesian Controller                      │
│  ┌──────────────┐     ┌──────────────┐   ┌──────────────┐ │
│  │ Pose         │────►│ Velocity     │──►│ UR5e Robot   │ │
│  │ Broadcasters │     │ Computation  │   │              │ │
│  └──────────────┘     └──────────────┘   └──────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Package Structure

```
se3_tcp_interface/
├── ros2_controllers_cartesian/     # Custom Cartesian motion controller
│   ├── include/                     # Controller header files
│   ├── src/                         # Controller implementation
│   └── cartesian_motion_controller_plugin.xml
│
├── se3_sensor_driver/               # SE3 sensor hardware interface
│   ├── include/                     # Hardware interface headers
│   ├── src/
│   │   ├── hardware_interface.cpp  # TCP client & ROS2 control interface
│   │   ├── sensor_tcp_server.cpp   # TCP server simulating sensor
│   │   └── tcp_server_node.cpp     # Server node executable
│   ├── urdf/                        # Sensor XACRO definitions
│   └── config/                      # Controller configurations
│
└── ur5e_cartesian_control/          # UR5e integration package
    ├── launch/                      # Launch file for complete system
    ├── config/                      # Controller parameters
    ├── urdf/                        # Combined robot description
    └── rviz/                        # Visualization config
```

## Key Features

### 1. **Cartesian Motion Controller**
- Implements velocity-based Cartesian control using KDL (Kinematics and Dynamics Library)
- Computes joint velocities via Weighted Damped Least Squares (WDLS) inverse kinematics
- Real-time pose tracking with automatic singularity detection

### 2. **SE3 Sensor Hardware Interface**
- Custom hardware interface implementing both `SensorInterface` and `IgnitionSystemInterface`
- TCP/IP communication for receiving pose data from external sensor
- Dual pose tracking: tool pose and target pose
- Serialized message passing using ROS2 serialization

### 3. **TCP Server (Sensor Simulator)**
- Simulates an external SE3 sensor system
- Alternately sends tool and target poses over TCP
- TF lookup-based pose generation for testing

### 4. **Transform Chain**
The system maintains the following transform relationships:
```
world → base → sensor_frame → {tool_sensor_pose, target_sensor_pose}
                ↓
         flange → tool_frame → tool_sensor
                ↓
         target_frame → target_sensor
```

## How It Works

### Control Flow

1. **Sensor Data Acquisition**
   - TCP server sends serialized `PoseStamped` messages for tool and target
   - Hardware interface receives and deserializes poses
   - State interfaces expose pose data to ROS2 control

2. **Pose Broadcasting**
   - Two pose broadcasters publish tool and target poses
   - Transforms are published to TF tree for visualization

3. **Cartesian Control Loop**
   - Controller reads current joint positions
   - Computes tool-to-base and target-to-base transforms using KDL
   - Calculates pose error (twist) between current and desired poses
   - Uses WDLS inverse kinematics to compute joint velocities
   - Sends velocity commands to robot joints

4. **Safety Features**
   - Singularity detection (velocities > 10 rad/s)
   - NaN checking for computed velocities
   - Motion threshold detection for reaching target
   - Velocity scaling factor (0.3) for safety

## Installation

### Prerequisites
- ROS2 Humble
- UR Robot Driver
- Ignition Gazebo (optional, for simulation)

### Build
```bash
cd ~/ros2_ws/src
git clone <repository_url> se3_tcp_interface
cd ~/ros2_ws
colcon build --packages-select ros2_controllers_cartesian se3_sensor_driver ur5e_cartesian_control
source install/setup.bash
```

## Usage

### Launch with Fake Hardware (Testing)
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py use_fake:=true
```

### Launch with Real UR5e
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py \
  robot_ip:=<YOUR_ROBOT_IP> \
  use_fake:=false
```

### Launch with Ignition Gazebo
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py use_gazebo:=true
```

### Adjusting Target Position
```bash
ros2 launch ur5e_cartesian_control ur5e.launch.py \
  x:=0.3 y:=-0.5 z:=0.6
```

## Parameters

### Controller Parameters (`assignment9_controllers.yaml`)
- `update_rate`: 500 Hz - Control loop frequency
- `joints`: List of UR5e joint names
- `robot_base_link`: Base frame for kinematics
- `end_effector_link`: End effector frame (flange)
- `sensor_link`: External sensor frame
- `interface_name`: velocity - Type of command interface

### Sensor Parameters
- `ip_address`: TCP server address (default: 127.0.0.1)
- `port`: 12345 - TCP communication port
- `sensor_id`: Identifier for tool/target sensor

## Technical Details

### KDL Chain Configuration
The controller builds a kinematic chain from `base` to `flange` using the URDF robot description. This chain is used for:
- Forward kinematics computation
- Inverse velocity kinematics (WDLS solver)

### Pose Error Computation
```cpp
KDL::Twist frame_error = KDL::diff(flange_to_base, target_to_base);
```
This computes the required twist (linear + angular velocity) to move from current pose to target.

### TCP Communication Protocol
- Message format: Serialized `geometry_msgs::msg::PoseStamped`
- Alternating pattern: tool pose → target pose → tool pose...
- Non-blocking reads with select() for real-time performance

## Visualization

The system includes RViz configuration showing:
- UR5e robot model
- Tool and target frames
- Transform tree
- Real-time pose updates

## Troubleshooting

### Common Issues

1. **"Failed to connect to TCP server"**
   - Ensure TCP server is running before launching controllers
   - Check IP address configuration

2. **"Possible singularity or collision"**
   - Robot near kinematic singularity
   - Adjust target position or approach angle

3. **"TF for TOOL/TARGET not available"**
   - Controllers not yet initialized
   - Wait for system startup completion

## Advanced Features

### Multi-Sensor Support
The hardware interface supports multiple sensors through parameterized sensor arrays, allowing expansion to multi-camera or multi-sensor setups.

### Ignition Gazebo Integration
Full simulation support with synchronized sensor data and robot control in Ignition Gazebo environment.

### Real-time Performance
- 500 Hz control rate
- Non-blocking TCP communication
- Optimized transform computations
