# Aurora NDI ROS2 Driver

A Docker-containerized ROS2 package for real-time electromagnetic tracking using the NDI Aurora system.

## Table of Contents

* [Overview](#overview)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Quick Start](#quick-start)
* [ROS2 Package](#ros2-package)
* [Configuration](#configuration)
  * [Multi-Sensor Setup](#multi-sensor-setup)
  * [Autoconfigured Tools (NDI Pens)](#autoconfigured-tools-ndi-pens)
  * [Parameter Reference](#parameter-reference)
* [Usage](#usage)
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)
* [License](#license)

## Overview

This system provides direct communication with the NDI Aurora electromagnetic tracking system through a fully containerized ROS2 environment. The package enables:

* Real-time Aurora electromagnetic position tracking
* Multi-sensor support (up to 4 sensors)
* Support for autoconfigured tools (NDI pens with internal ROM)
* TF2 transform broadcasting for robot/surgical navigation
* Configurable tracking parameters via YAML files
* Data filtering and quality monitoring
* Robust error handling and automatic reconnection

The entire system is containerized using Docker for easy deployment and reproducibility.

## Prerequisites

* Docker (version 20.10 or higher)
* Docker Compose (version 2.0 or higher)
* NDI Aurora electromagnetic tracking system
* USB serial connection (typically `/dev/ttyUSB0`)

## Installation

1. Clone the repository:

```bash
git clone https://github.com/yourusername/aurora-ndi-ros2-docker.git
cd aurora-ndi-ros2-docker
```

2. Build the Docker image:

```bash
docker build -t aurora_ndi_ros2:latest .
```

3. Configure X11 for GUI tools (RViz2, rqt):

```bash
xhost +local:root
```

## Quick Start

1. Start the container:

```bash
docker compose up
```

2. Open an additional shell in the container:

```bash
docker exec -it aurora_ndi_tracker bash
```

3. The Aurora node starts automatically via the entrypoint script

## ROS2 Package

The system operates within a ROS2 Humble environment and publishes the following coordinate frames:

* `aurora_base`: World reference frame (Aurora base station)
* `sensor0`, `sensor1`, ... : Aurora sensor tracking frames (tools being tracked)

### TF Tree Structure

```
world
  └── aurora_base
        ├── endo_aurora_sensor0
        ├── endo_aurora_sensor1
        └── endo_aurora_pen
```

### Published Topics

* `/aurora_data_sensor0` (aurora_pub/msg/AuroraData)
* `/aurora_data_sensor1` (aurora_pub/msg/AuroraData)
* `/aurora_data_pen` (aurora_pub/msg/AuroraData)

Message structure:

```
std_msgs/Header header
geometry_msgs/Point position      # Position in mm
geometry_msgs/Quaternion orientation
float64 error
bool visible
uint16 port_handle
```

## Configuration

The Aurora driver is configured through YAML parameter files located in `config/`. The driver supports multiple sensors and can handle both regular sensors (with external ROM files) and autoconfigured tools like NDI pens (with internal ROM).

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
| `measure_reply_option` | string | `"0001"` | Measurement reply option bitmask:<br>- `0001`: Transform data<br>- `0002`: Tool & marker info<br>- `0004`: 3D pos of 1 active marker<br>- `0008`: 3D pos of markers on tool<br>- `0800`: Transform not normally reported<br>- `1000`: 3D pos of stray passive markers |
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
  - "/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
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

#### Example 3: Two Regular Sensors

```yaml
num_sensors: 2
tool_rom_files:
  - "/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
  - "/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
port_handles: ["0A", "0B"]
port_handles_autoconfig: [""]  # No autoconfigured sensors
topic_names: 
  - "/aurora_data_sensor0"
  - "/aurora_data_sensor1"
child_frame_names:
  - "endo_aurora_sensor0"
  - "endo_aurora_sensor1"
```

#### Example 4: Mixed Configuration (1 Sensor + 1 Pen)

```yaml
num_sensors: 2
tool_rom_files:
  - "/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"  # Only for 0A
port_handles: ["0A", "0B"]  # Both ports
port_handles_autoconfig: ["0B"]  # Only 0B is autoconfigured
topic_names:
  - "/aurora_data_sensor0"
  - "/aurora_data_pen"
child_frame_names:
  - "endo_aurora_sensor0"
  - "endo_aurora_pen"
```

#### Example 5: Multiple Sensors with Multiple Pens

```yaml
num_sensors: 4
tool_rom_files:
  - "/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"  # For 0A
  - "/workspace/src/aurora_pub/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"  # For 0B
port_handles: ["0A", "0B", "0C", "0D"]
port_handles_autoconfig: ["0C", "0D"]  # Ports 0C and 0D are pens
topic_names:
  - "/aurora_data_sensor0"
  - "/aurora_data_sensor1"
  - "/aurora_data_pen1"
  - "/aurora_data_pen2"
child_frame_names:
  - "endo_aurora_sensor0"
  - "endo_aurora_sensor1"
  - "endo_aurora_pen1"
  - "endo_aurora_pen2"
```

## Usage

### Running Aurora Tracking

The Aurora node is launched automatically when the container starts. To manually control it:

Launch Aurora tracking:

```bash
ros2 launch aurora_pub aurora_pub.launch.py
```

Run the node directly:

```bash
ros2 run aurora_pub aurora_publisher_node
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

### Aurora Device Not Found

If you get connection errors:

1. Check device connection:

```bash
ls -l /dev/ttyUSB*
```

2. Verify permissions:

```bash
sudo chmod 666 /dev/ttyUSB0
```

3. Check if device is mapped in compose:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

### No TF Data Published

1. Verify Aurora tracking is active:

```bash
ros2 topic list | grep aurora
```

2. Check node status:

```bash
ros2 node list
ros2 node info /aurora_publisher_node
```

3. Verify sensor visibility in published data:

```bash
ros2 topic echo /aurora_data_sensor0 --field visible
```

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

### Performance Issues

If tracking is slow or laggy:

1. Reduce publish rate:

```yaml
publish_rate_hz: 20.0  # Instead of 40.0
```

2. Disable filtering:

```yaml
enable_data_filtering: false
```

3. Reduce filter window:

```yaml
filter_window_size: 2  # Instead of 4
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

Property of Edoardo Guida, student at Politecnico di Milano, developed at NEARlab