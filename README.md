# Aurora NDI ROS2 Driver

A ROS2 Humble package for real-time electromagnetic tracking using the NDI Aurora system.

## Table of Contents

* [Overview](#overview)
* [Branches](#branches)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [ROS2 Package Structure](#ros2-package-structure)
* [Configuration](#configuration)
  * [Multi-Sensor Setup](#multi-sensor-setup)
  * [Autoconfigured Tools (NDI Pens)](#autoconfigured-tools-ndi-pens)
  * [Parameter Reference](#parameter-reference)
* [Usage](#usage)
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)
* [License](#license)

## Overview

This package provides direct communication with the NDI Aurora electromagnetic tracking system through ROS2 Humble. The package enables:

* Real-time Aurora electromagnetic position tracking
* Multi-sensor support (up to 4 sensors)
* Support for autoconfigured tools (NDI pens with internal ROM)
* TF2 transform broadcasting for robot/surgical navigation
* Configurable tracking parameters via YAML files

## Branches

This repository contains different branches for different use cases:

* **`ros2-package`** (current): Pure ROS2 package - use this to integrate Aurora tracking into your existing ROS2 workspace
* **`docker-setup`**: Complete Docker containerized setup with example configuration - use this for quick start and testing

**🐳 Want a Docker setup?** Switch to the `docker-setup` branch:

```bash
git checkout docker-setup
```

The `docker-setup` branch includes:
- Pre-configured Dockerfile
- Docker Compose setup
- Full documentation for containerized deployment

## Prerequisites

* ROS2 Humble (Ubuntu 22.04)
* NDI Aurora electromagnetic tracking system
* USB serial connection (typically `/dev/ttyUSB0`)
* Build tools: `colcon`, `cmake`, `gcc/g++`

### System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
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

## ROS2 Package Structure

```
aurora_ndi_ros2_driver/
├── config/
│   └── aurora_config.yaml          # Configuration parameters
├── include/
│   └── aurora_ndi_ros2_driver/
│       ├── aurora_publisher_node.hpp
│       ├── aurora_utils.hpp
│       └── ndi_aurora_ros2.hpp
├── launch/
│   └── aurora_pub.launch.py        # Launch file
├── msg/
│   └── AuroraData.msg              # Custom message definition
├── rom/
│   └── 610029_AuroraMini6DOF_1.8x9mm/
│       └── 610029-6DOF.rom         # Tool ROM files
├── src/
│   ├── aurora_publisher_node.cpp
│   ├── aurora_utils.cpp
│   └── ndi_aurora_ros2.cpp
├── CMakeLists.txt
└── package.xml
```

### Published Topics

* `/aurora_data_sensor0` (aurora_ndi_ros2_driver/msg/AuroraData)
* `/aurora_data_sensor1` (aurora_ndi_ros2_driver/msg/AuroraData)
* `/aurora_data_pen` (aurora_ndi_ros2_driver/msg/AuroraData)

### AuroraData Message Structure

```
std_msgs/Header header
geometry_msgs/Point position      # Position in mm
geometry_msgs/Quaternion orientation
float64 error
bool visible
uint16 port_handle
```

### TF Tree Structure

```
aurora_base
  ├── endo_aurora_sensor0
  ├── endo_aurora_sensor1
  └── endo_aurora_pen
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
| `enable_data_filtering` | bool | `true` | Enable moving average filter for position and orientation |
| `filter_window_size` | int | `4` | Number of samples for moving average filter |
| `position_scale_factor` | double | `1.0` | Scale factor applied to position measurements |
| `orientation_scale_factor` | double | `1.0` | Scale factor applied to orientation quaternions |
| `error_scale_factor` | double | `1.0` | Scale factor applied to error measurements |

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
  - "$(find aurora_ndi_ros2_driver)/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
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
  - "$(find aurora_ndi_ros2_driver)/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"  # Only for 0A
port_handles: ["0A", "0B"]  # Both ports
port_handles_autoconfig: ["0B"]  # Only 0B is autoconfigured
topic_names:
  - "/aurora_data_sensor0"
  - "/aurora_data_pen"
child_frame_names:
  - "endo_aurora_sensor0"
  - "endo_aurora_pen"
```

## Usage

### Running Aurora Tracking

Launch Aurora tracking with default configuration:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py
```

Launch with custom configuration file:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_pub.launch.py config_file:=/path/to/your/config.yaml
```

Run the node directly:

```bash
ros2 run aurora_ndi_ros2_driver aurora_publisher_node
```

### Device Permissions

Ensure you have permissions to access the USB device:

```bash
sudo chmod 666 /dev/ttyUSB0
```

Or add your user to the dialout group (recommended):

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### Visualization

Visualize the coordinate frames and transforms using RViz2:

```bash
ros2 run rviz2 rviz2
```

Add the TF display plugin to see the `aurora_base` → `sensor` transforms.

### Monitoring Data

View published Aurora data:

```bash
ros2 topic echo /aurora_data_sensor0
```

Check TF transforms:

```bash
ros2 run tf2_ros tf2_echo aurora_base endo_aurora_sensor0
```

Visualize TF tree:

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

List all available topics:

```bash
ros2 topic list
```

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

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

Property of Edoardo Guida, student at Politecnico di Milano, developed at NEARlab

---

**Need Docker?** Check out the [`docker-setup`](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/docker-setup) branch for a complete containerized setup.