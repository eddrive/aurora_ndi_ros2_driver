# Aurora NDI ROS2 Driver

[![ROS2 Version](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)

A ROS2 Humble package for real-time electromagnetic tracking using the NDI Aurora system with direct serial communication.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Branches](#branches)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Configuration](#configuration)
  - [Multi-Sensor Setup](#multi-sensor-setup)
  - [Autoconfigured Tools (NDI Pens)](#autoconfigured-tools-ndi-pens)
  - [Parameter Reference](#parameter-reference)
  - [Configuration Examples](#configuration-examples)
- [Usage](#usage)
  - [Quick Start](#quick-start)
  - [Launch File Options](#launch-file-options)
  - [Device Permissions](#device-permissions)
  - [Visualization](#visualization)
- [ROS2 Interface](#ros2-interface)
  - [Published Topics](#published-topics)
  - [Message Definitions](#message-definitions)
  - [TF Frames](#tf-frames)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Overview

This package provides direct serial communication with the NDI Aurora electromagnetic tracking system through ROS2 Humble. It enables real-time 6-DOF pose tracking for surgical navigation, robotic guidance, and medical device tracking applications.

## Features

- **Real-time Tracking**: High-frequency position and orientation tracking (up to 40 Hz)
- **Multi-Sensor Support**: Track up to 4 sensors simultaneously
- **Autoconfigured Tools**: Support for NDI pens with internal ROM configuration
- **TF2 Integration**: Automatic transform broadcasting for seamless ROS2 integration
- **Flexible Configuration**: YAML-based parameter configuration
- **Data Filtering**: Optional moving average filter and outlier detection
- **Robust Communication**: Automatic reconnection with configurable retries
- **Docker Ready**: Optional Docker setup available on separate branch

## Branches

This repository contains two main branches for different deployment scenarios:

| Branch | Purpose | Use Case |
|--------|---------|----------|
| **`ros2-package`** (current) | Pure ROS2 package | Integration into existing ROS2 workspace |
| **`docker-setup`** | Docker containerized setup | Quick start, testing, and isolated deployment |

### Docker Setup

For a containerized deployment with pre-configured environment:

```bash
git checkout docker-setup
```

The `docker-setup` branch includes:
- Pre-built Docker image with all dependencies
- Docker Compose configuration
- Example launch configurations
- Isolated testing environment

## Prerequisites

### Hardware Requirements

- NDI Aurora electromagnetic tracking system
- USB-to-serial cable (typically connects as `/dev/ttyUSB0`)
- Aurora field generator and compatible sensors/tools

### Software Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Build Tools**: `colcon`, `cmake` (≥3.8), `gcc/g++`

### System Dependencies

Install ROS2 Humble following the [official installation guide](https://docs.ros.org/en/humble/Installation.html), then install package dependencies:

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libserial-dev \
    python3-serial
```

## Installation

### 1. Create ROS2 Workspace (if you don't have one)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/eddrive/aurora_ndi_ros2_driver.git
cd aurora_ndi_ros2_driver
git checkout ros2-package  # Ensure you're on the ros2-package branch
```

### 3. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select aurora_ndi_ros2_driver
source install/setup.bash
```

## Package Structure

```
aurora_ndi_ros2_driver/
├── config/
│   └── aurora_config.yaml          # Configuration parameters
├── include/
│   └── aurora_ndi_ros2_driver/
│       ├── aurora_publisher_node.hpp  # Main node header
│       ├── aurora_utils.hpp           # Utility functions
│       └── ndi_aurora_ros2.hpp        # Aurora driver interface
├── launch/
│   └── aurora_pub.launch.py        # Launch file with arguments
├── msg/
│   └── AuroraData.msg              # Custom message definition
├── rom/
│   └── 610029_AuroraMini6DOF_1.8x9mm/
│       └── 610029-6DOF.rom         # Sensor ROM files
├── src/
│   ├── aurora_publisher_node.cpp   # ROS2 node implementation
│   ├── aurora_utils.cpp            # Helper utilities
│   └── ndi_aurora_ros2.cpp         # Aurora communication driver
├── CMakeLists.txt                  # Build configuration
├── package.xml                     # Package manifest
└── README.md                       # This file
```

## Configuration

The Aurora driver is configured through the YAML parameter file in `config/aurora_config.yaml`. The driver supports multiple sensors and can handle both regular sensors (with external ROM files) and autoconfigured tools like NDI pens (with internal ROM).

### Multi-Sensor Setup

The driver supports up to 4 sensors simultaneously. Each sensor can be:
- A **regular sensor** that requires an external ROM file
- An **autoconfigured tool** (e.g., NDI pen) that has an internal ROM

#### Configuration Logic

```yaml
num_sensors: <total number of sensors>
tool_rom_files: [<ROM files for regular sensors only>]
port_handles: [<all port handles>]
port_handles_autoconfig: [<which ports are autoconfigured>]
```

**Important**: The number of ROM files must equal: `num_sensors - number of autoconfigured ports`

### Autoconfigured Tools (NDI Pens)

NDI pens and similar tools have built-in ROM data and don't need external ROM files. To use them:

1. Add the port handle to `port_handles` array
2. Add the same port handle to `port_handles_autoconfig` array
3. Do NOT include a ROM file for this port

**Example**: Single NDI pen on port 0A:

```yaml
num_sensors: 1
tool_rom_files: [""]  # Empty placeholder (no ROM needed)
port_handles: ["0A"]
port_handles_autoconfig: ["0A"]
```

### Parameter Reference

#### Serial Communication

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyUSB0` | Serial port device for Aurora connection |
| `baud_rate` | int | `230400` | Communication baud rate. Supported: 9600, 19200, 38400, 57600, 115200, 230400 |
| `serial_timeout_sec` | double | `2.0` | Timeout for serial communication operations (seconds) |

#### Multi-Sensor Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_sensors` | int | `1` | Total number of sensors (1-4), including both regular and autoconfigured |
| `tool_rom_files` | string[] | `["...6DOF.rom"]` | ROM files for regular sensors. Use `[""]` if all sensors are autoconfigured. Length must be `num_sensors - num_autoconfig` |
| `port_handles` | string[] | `["0A"]` | Port handles for ALL sensors (hex: 0A, 0B, 0C, 0D). Must have `num_sensors` entries |
| `port_handles_autoconfig` | string[] | `[]` | Port handles for autoconfigured tools (NDI pens). Use `[""]` if no autoconfigured tools |
| `topic_names` | string[] | `["/aurora_data_sensor0"]` | ROS2 topic names for each sensor. Must have `num_sensors` entries |
| `child_frame_names` | string[] | `["endo_aurora_sensor0"]` | TF frame names for each sensor. Must have `num_sensors` entries |

#### Aurora System Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `reference_port` | string | `"10"` | Reference port handle for tracking system |
| `track_reply_option` | int | `80` | Tracking reply option: `0` (no reset) or `80` (reset frame counter) |
| `measure_reply_option` | string | `"0001"` | Measurement reply option bitmask |
| `status_reply` | bool | `true` | Include status information in tracking replies |

#### ROS Publishing Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `"aurora_base"` | Parent frame ID for all sensor transforms |
| `publish_rate_hz` | double | `40.0` | Publishing frequency for data and transforms (Hz) |
| `queue_size` | int | `10` | ROS2 publisher queue size |

#### Data Processing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_data_filtering` | bool | `false` | Enable moving average filter for position and orientation |
| `filter_window_size` | int | `4` | Number of samples for moving average filter (introduces temporal delay) |
| `enable_outlier_detection` | bool | `false` | Enable outlier rejection for physically implausible measurements |
| `max_position_jump_mm` | double | `20.0` | Maximum allowed position change between samples (mm) |
| `max_orientation_change_deg` | double | `30.0` | Maximum allowed orientation change between samples (degrees) |
| `max_acceptable_error_mm` | double | `3.0` | Maximum acceptable RMS error from Aurora (mm) |
| `position_scale_factor` | double | `1.0` | Scale factor applied to position measurements |
| `orientation_scale_factor` | double | `1.0` | Scale factor applied to orientation quaternions |
| `error_scale_factor` | double | `1.0` | Scale factor applied to error measurements |

> **Note**: `enable_data_filtering` introduces temporal delay. For time-critical applications (e.g., hand-eye calibration), use `enable_outlier_detection` instead, which rejects invalid samples without delay.

#### Connection and Timeouts

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `command_timeout_sec` | double | `2.0` | Timeout for Aurora command responses (seconds) |
| `max_connection_retries` | int | `3` | Maximum number of connection retry attempts |
| `retry_delay_sec` | double | `2.0` | Delay between connection retry attempts (seconds) |

#### Logging

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `debug_mode` | bool | `false` | Enable verbose debug logging |
| `log_raw_data` | bool | `false` | Log raw Aurora measurement data |

### Configuration Examples

#### Example 1: Single Sensor with ROM File

```yaml
num_sensors: 1
tool_rom_files:
  - "/path/to/your/workspace/src/aurora_ndi_ros2_driver/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
port_handles: ["0A"]
port_handles_autoconfig: [""]  # No autoconfigured sensors
topic_names: ["/aurora_data_sensor0"]
child_frame_names: ["endo_aurora_sensor0"]
```

#### Example 2: Single NDI Pen (Autoconfigured)

```yaml
num_sensors: 1
tool_rom_files: [""]  # Empty - pen has internal ROM
port_handles: ["0A"]
port_handles_autoconfig: ["0A"]  # Port 0A is autoconfigured
topic_names: ["/aurora_data_pen"]
child_frame_names: ["endo_aurora_pen"]
```

#### Example 3: Mixed Configuration (1 Sensor + 1 Pen)

```yaml
num_sensors: 2
tool_rom_files:
  - "/path/to/your/workspace/src/aurora_ndi_ros2_driver/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
port_handles: ["0A", "0B"]
port_handles_autoconfig: ["0B"]  # Only 0B is autoconfigured
topic_names:
  - "/aurora_data_sensor0"
  - "/aurora_data_pen"
child_frame_names:
  - "endo_aurora_sensor0"
  - "endo_aurora_pen"
```

> **Note**: Replace `/path/to/your/workspace` with your actual ROS2 workspace path (e.g., `~/ros2_ws`).

## Usage

### Quick Start

1. **Ensure Aurora is connected** via USB (typically `/dev/ttyUSB0`)
2. **Configure device permissions** (see [Device Permissions](#device-permissions))
3. **Launch the driver** with default configuration:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py
```

### Launch File Options

The launch file supports several arguments for flexible deployment:

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `config/aurora_config.yaml` | Path to configuration YAML file |
| `debug` | `false` | Enable debug mode with verbose logging |
| `log_level` | `info` | ROS logging level (`debug`, `info`, `warn`, `error`) |
| `use_rviz` | `false` | Launch RViz2 for visualization |
| `publish_tf` | `true` | Publish static transform from `world` to `aurora_base` |
| `namespace` | ` ` | Optional ROS namespace for the node |

#### Examples

Launch with custom configuration:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py \
    config_file:=/path/to/your/config.yaml
```

Launch with RViz visualization:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py \
    use_rviz:=true
```

Launch in debug mode:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py \
    debug:=true \
    log_level:=debug
```

Run the node directly (without launch file):

```bash
ros2 run aurora_ndi_ros2_driver aurora_publisher_node --ros-args \
    --params-file ~/ros2_ws/src/aurora_ndi_ros2_driver/config/aurora_config.yaml
```

### Device Permissions

The Aurora device requires proper USB permissions. Choose one of these methods:

#### Option 1: Add User to dialout Group (Recommended)

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in for changes to take effect.

#### Option 2: Temporary Permission (Testing Only)

```bash
sudo chmod 666 /dev/ttyUSB0
```

> **Note**: This needs to be repeated after each device reconnection.

### Visualization

Launch with RViz2 for real-time visualization:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py use_rviz:=true
```

Or launch RViz2 separately:

```bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set **Fixed Frame** to `world` or `aurora_base`
2. Add **TF** display to visualize coordinate frames
3. Add **Axes** display to see sensor orientations

### Monitoring Data

Monitor published sensor data:

```bash
# View data from sensor 0
ros2 topic echo /aurora_data_sensor0

# Monitor publishing frequency
ros2 topic hz /aurora_data_sensor0

# View all Aurora topics
ros2 topic list | grep aurora
```

Monitor TF transforms:

```bash
# View transform between frames
ros2 run tf2_ros tf2_echo aurora_base endo_aurora_sensor0

# Visualize TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# View all available frames
ros2 run tf2_ros tf2_monitor
```

## ROS2 Interface

### Published Topics

The driver publishes tracking data on topics configured in the YAML file. Default topics:

| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/aurora_data_sensor0` | `aurora_ndi_ros2_driver/AuroraData` | Sensor 0 tracking data |
| `/aurora_data_sensor1` | `aurora_ndi_ros2_driver/AuroraData` | Sensor 1 tracking data (if configured) |
| `/aurora_data_pen` | `aurora_ndi_ros2_driver/AuroraData` | NDI pen tracking data (if configured) |

> **Note**: Topic names are configurable via the `topic_names` parameter in the configuration file.

### Message Definitions

#### AuroraData.msg

```
std_msgs/Header header
geometry_msgs/Point position        # Position in mm (x, y, z)
geometry_msgs/Quaternion orientation # Orientation as quaternion (x, y, z, w)
float64 error                       # RMS tracking error in mm
bool visible                        # Sensor visibility status
uint32 port_handle                  # Port handle (decimal representation)
```

**Field Details:**

- `header.stamp`: Timestamp of the measurement
- `header.frame_id`: Parent frame (typically `aurora_base`)
- `position`: 3D position in millimeters relative to the field generator
- `orientation`: 3D orientation as a unit quaternion
- `error`: Root Mean Square (RMS) tracking error reported by Aurora
- `visible`: `true` if sensor is visible to the field generator, `false` otherwise
- `port_handle`: Numeric port handle identifier

### TF Frames

The driver publishes TF transforms for each configured sensor:

```
world (optional, via static_transform_publisher)
  └── aurora_base (field generator reference frame)
      ├── endo_aurora_sensor0 (sensor 0)
      ├── endo_aurora_sensor1 (sensor 1, if configured)
      └── endo_aurora_pen (NDI pen, if configured)
```

**Frame Naming:**
- Parent frame: Configured via `frame_id` parameter (default: `aurora_base`)
- Child frames: Configured via `child_frame_names` parameter

**Transform Publishing:**
- **Rate**: Configured via `publish_rate_hz` (default: 40 Hz)
- **Broadcaster**: Uses `tf2_ros::TransformBroadcaster`
- **Coordinate System**: Right-handed, follows ROS REP-103 conventions

## Troubleshooting

### Configuration Errors

#### Error: "tool_rom_files size doesn't match expected"

**Cause**: Mismatch between ROM files and sensor configuration.

**Solution**: Ensure `tool_rom_files` length equals `num_sensors - num_autoconfig`:

```yaml
# For 2 sensors where 1 is autoconfigured:
num_sensors: 2
tool_rom_files: ["sensor.rom"]  # Only 1 ROM file
port_handles_autoconfig: ["0B"]  # 1 autoconfigured
```

#### Error: "Autoconfig port 'XX' not found in port_handles list"

**Cause**: Port listed in `port_handles_autoconfig` is not in `port_handles`.

**Solution**: Ensure all autoconfigured ports are also in the main port list:

```yaml
port_handles: ["0A", "0B"]  # Must include ALL ports
port_handles_autoconfig: ["0B"]  # Subset of port_handles
```

#### Error: "Failed to initialize port handles"

**Cause**: Aurora hardware cannot initialize the specified ports.

**Solution**: 
1. Check physical connections to Aurora
2. Verify correct ROM files are specified
3. Ensure ports are not already in use
4. Check Aurora system status LEDs

### Hardware and Connection Issues

#### Error: "Failed to open serial port"

**Cause**: Cannot access `/dev/ttyUSB0` or device not found.

**Solutions**:
1. Check device connection:
   ```bash
   ls -l /dev/ttyUSB*
   dmesg | grep tty
   ```
2. Verify permissions (see [Device Permissions](#device-permissions))
3. Check if device path is correct in config file
4. Ensure no other process is using the port:
   ```bash
   sudo fuser /dev/ttyUSB0
   ```

#### Error: "Serial timeout" or "No response from Aurora"

**Cause**: Communication issues with Aurora device.

**Solutions**:
1. Check baud rate matches Aurora configuration (default: 230400)
2. Power cycle the Aurora system
3. Check USB cable connection
4. Try different baud rates: 115200, 57600
5. Increase `serial_timeout_sec` and `command_timeout_sec` in config

#### Error: "Sensor not visible" (visible: false)

**Cause**: Sensor is out of tracking volume or obstructed.

**Solutions**:
1. Check sensor is within Aurora field generator tracking volume
2. Remove metallic objects from tracking area
3. Ensure sensor cable is not damaged
4. Verify ROM file matches the physical sensor
5. Check field generator is powered on and initialized

### Build Errors

If you encounter build errors:

1. Ensure all dependencies are installed:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Clean build and rebuild:
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build --packages-select aurora_ndi_ros2_driver
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### Runtime Issues

#### Node crashes or restarts frequently

**Possible causes**:
- USB connection intermittent
- Aurora device power issues
- Invalid configuration parameters

**Solutions**:
1. Check system logs:
   ```bash
   ros2 run aurora_ndi_ros2_driver aurora_publisher_node --ros-args --log-level debug
   ```
2. Verify configuration file syntax
3. Check USB cable quality
4. Enable node respawn in launch file (already enabled by default)

#### Low publishing rate or data latency

**Solutions**:
1. Increase `publish_rate_hz` in config (max ~60 Hz for Aurora)
2. Disable data filtering if not needed
3. Check system CPU usage
4. Reduce number of active sensors

## Contributing

Contributions are welcome! If you find bugs, have feature requests, or want to improve the documentation, please:

1. **Fork** the repository
2. **Create** a feature branch:
   ```bash
   git checkout -b feature/amazing-feature
   ```
3. **Commit** your changes:
   ```bash
   git commit -m 'Add amazing feature'
   ```
4. **Push** to the branch:
   ```bash
   git push origin feature/amazing-feature
   ```
5. **Open** a Pull Request

### Development Guidelines

- Follow [ROS2 coding style guidelines](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Ensure code compiles without warnings
- Test changes with real Aurora hardware when possible
- Update documentation for new features

## License

**Copyright © 2024 Edoardo Guida**

Property of Edoardo Guida, student at Politecnico di Milano.
Developed at NEARlab - NearLab Politecnico di Milano.

## Acknowledgments

- **Institution**: [Politecnico di Milano](https://www.polimi.it/)
- **Laboratory**: [NEARlab - NearLab Polimi](https://nearlab.polimi.it/)
- **Developer**: Edoardo Guida ([edoardo1.guida@mail.polimi.it](mailto:edoardo1.guida@mail.polimi.it))
- **NDI**: Northern Digital Inc. for Aurora electromagnetic tracking technology

### References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [NDI Aurora Technical Documentation](https://www.ndigital.com/products/aurora/)
- [TF2 Documentation](https://docs.ros.org/en/humble/Concepts/About-Tf2.html)

---

**Need Docker?** Check out the [`docker-setup`](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/docker-setup) branch for a complete containerized setup.

**Questions or Issues?** Please [open an issue](https://github.com/eddrive/aurora_ndi_ros2_driver/issues) on GitHub.