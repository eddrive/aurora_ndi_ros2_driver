# Aurora NDI ROS2 Driver

A Docker-containerized ROS2 package for real-time electromagnetic tracking using the NDI Aurora system.

## Table of Contents

* [Overview](#overview)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Quick Start](#quick-start)
* [ROS2 Package](#ros2-package)
* [Usage](#usage)
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)
* [License](#license)

## Overview

This system provides direct communication with the NDI Aurora electromagnetic tracking system through a fully containerized ROS2 environment. The package enables:

* Real-time Aurora electromagnetic position tracking
* TF2 transform broadcasting for robot/surgical navigation
* Configurable tracking parameters via YAML files
* Data filtering and quality monitoring
* Robust error handling and automatic reconnection

The entire system is containerized using Docker for easy deployment and reproducibility.

## Prerequisites

* **Docker** (version 20.10 or higher)
* **Docker Compose** (version 2.0 or higher)
* **NDI Aurora electromagnetic tracking system**
* **USB serial connection** (typically `/dev/ttyUSB0`)

## Installation

1. **Clone the repository:**

```bash
git clone https://github.com/yourusername/aurora-ndi-ros2-docker.git
cd aurora-ndi-ros2-docker
```

2. **Build the Docker image:**

```bash
docker build -t aurora_ndi_ros2:latest .
```

3. **Configure X11 for GUI tools (RViz2, rqt):**

```bash
xhost +local:root
```

## Quick Start

1. **Start the container:**

```bash
docker compose up
```

2. **Open an additional shell in the container:**

```bash
docker exec -it aurora_ndi_tracker bash
```

3. **The Aurora node starts automatically via the entrypoint script**

## ROS2 Package

The system operates within a ROS2 Humble environment and publishes the following coordinate frames:

* **`aurora_base`**: World reference frame (Aurora base station)
* **`sensor0`**: Aurora sensor tracking frame (tool being tracked)

### TF Tree Structure

```
aurora_base
    └── sensor0
```

Publishes Aurora electromagnetic tracking sensor data to the ROS2 ecosystem using the `ndi_aurora` driver.

### Published Topics:

* **`/aurora_data`** (aurora_pub/msg/AuroraData):

```
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float64 error
bool visible
string port_handle
```

### Parameters:

Key configurable parameters (see `config/aurora_params.yaml`):

* `serial_port`: Aurora device serial port (default: `/dev/ttyUSB0`)
* `baud_rate`: Communication baud rate (default: `230400`)
* `tool_handle`: Primary tool handle (default: `"0A"`)
* `publish_rate_hz`: Publishing frequency (default: `40.0`)
* `enable_data_filtering`: Enable moving average filter (default: `true`)
* `filter_window_size`: Filter window size (default: `4`)

## Usage

### Running Aurora Tracking

The Aurora node is launched automatically when the container starts. To manually control it:

**Launch Aurora tracking:**

```bash
ros2 launch aurora_pub aurora_pub.launch.py
```

**Run the node directly:**

```bash
ros2 run aurora_pub aurora_publisher_node
```

### Visualization

Visualize the coordinate frames and transforms using RViz2:

```bash
ros2 run rviz2 rviz2
```

Add the TF display plugin to see the `aurora_base` → `sensor0` transform.

### Monitoring Data

**View published Aurora data:**

```bash
ros2 topic echo /aurora_data
```

**Check TF transforms:**

```bash
ros2 run tf2_ros tf2_echo aurora_base sensor0
```

**Visualize TF tree:**

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

## Troubleshooting

### Aurora Device Not Found

If you get connection errors:

1. **Check device connection:**
```bash
ls -l /dev/ttyUSB*
```

2. **Verify permissions:**
```bash
sudo chmod 666 /dev/ttyUSB0
```

3. **Check if device is mapped in compose:**
```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

### No TF Data Published

1. **Verify Aurora tracking is active:**
```bash
ros2 topic list | grep aurora
```

2. **Check node status:**
```bash
ros2 node list
ros2 node info /aurora_publisher_node
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

Property of Edoardo Guida, student at Politecnico di Milano, developed at NEARlab
