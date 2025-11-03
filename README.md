# Aurora NDI ROS2 Driver - Docker Setup

A fully containerized ROS2 Humble environment for real-time electromagnetic tracking using the NDI Aurora system.

## Table of Contents

* [Overview](#overview)
* [Branches](#branches)
* [Prerequisites](#prerequisites)
* [Quick Start](#quick-start)
* [Docker Setup](#docker-setup)
* [Configuration](#configuration)
* [Usage](#usage)
* [Troubleshooting](#troubleshooting)
* [Advanced Usage](#advanced-usage)
* [Contributing](#contributing)
* [License](#license)

## Overview

This branch provides a complete Docker containerized setup for the NDI Aurora tracking system. Everything you need is included:

* Pre-configured Docker environment with ROS2 Humble
* Automatic Aurora driver cloning and building from the `ros2-package` branch during Docker build
* GUI support for RViz2 and rqt tools
* Volume mounting for device access and X11 forwarding

**Perfect for:**
- Quick testing and evaluation
- Development without installing ROS2 locally
- Consistent environments across different machines
- Easy deployment in production

## Branches

This repository contains different branches for different use cases:

* **`ros2-package`**: Pure ROS2 package - use this to integrate Aurora tracking into your existing ROS2 workspace
* **`docker-setup`** (current): Complete Docker containerized setup - use this for quick start and testing

**🔧 Want just the ROS2 package?** Switch to the `ros2-package` branch:

```bash
git checkout ros2-package
```

## Prerequisites

* Docker (version 20.10 or higher)
* Docker Compose (version 2.0 or higher)
* NDI Aurora electromagnetic tracking system
* USB serial connection (typically `/dev/ttyUSB0`)

### Install Docker

If you don't have Docker installed:

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to docker group (avoid using sudo)
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt-get update
sudo apt-get install docker-compose-plugin

# Log out and log back in for changes to take effect
```

## Quick Start

### 1. Clone this branch

```bash
git clone -b docker-setup https://github.com/eddrive/aurora_ndi_ros2_driver.git
cd aurora_ndi_ros2_driver
```

### 2. Configure X11 for GUI tools (RViz2, rqt)

```bash
xhost +local:root
```

### 3. Build the Docker image

Build the image with cache busting to ensure you get the latest driver code:

```bash
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
```

**Note:** The `CACHEBUST` argument forces Docker to re-clone the latest version of the driver from the `ros2-package` branch.

### 4. Start the container

```bash
docker compose up
```

The Aurora node will start automatically!

### 5. Access the container (optional)

Open a new terminal and run:

```bash
docker exec -it aurora_ndi_tracker bash
```

Now you can use ROS2 commands, launch RViz2, etc.

## Docker Setup

### Project Structure

```
aurora_ndi_ros2_driver/ (docker-setup branch)
├── Dockerfile                      # Container definition with git clone
├── docker-compose.yml              # Service orchestration
├── entrypoint.sh                   # Container startup script
└── README.md                       # This file
```

**Note:** The Aurora driver package is automatically cloned from the `ros2-package` branch during the Docker build process. There is no local `resources/` or source code directory in this branch.

### Dockerfile Overview

The Docker image includes:
- Ubuntu 22.04 base
- ROS2 Humble Desktop (full installation)
- Aurora NDI driver (automatically cloned from `ros2-package` branch during build)
- Visualization tools (RViz2, rqt)
- Development tools (nano, vim, htop, tree)
- Pre-built workspace ready to use

The build process:
1. Installs all ROS2 dependencies
2. Clones the latest driver code from GitHub (`ros2-package` branch)
3. Builds the workspace with `colcon build`
4. Creates a ready-to-run container

### Docker Compose Configuration

The `docker-compose.yml` defines:
- Container name: `aurora_ndi_tracker`
- Network mode: `host` (for ROS2 communication)
- USB device mapping: `/dev/ttyUSB0`
- Display forwarding for GUI tools (`DISPLAY` and X11 socket)
- Privileged mode for full device access

## Configuration

The Aurora driver configuration is built into the container from the `ros2-package` branch. The default configuration file is located at:

**Location in container:** `/workspace/src/aurora_ndi_ros2_driver/config/aurora_config.yaml`

To modify the configuration, you have two options:

**Option 1: Edit inside the container**
```bash
docker exec -it aurora_ndi_tracker bash
nano /workspace/src/aurora_ndi_ros2_driver/config/aurora_config.yaml
# Restart the node for changes to take effect
```

**Option 2: Rebuild with custom config**
Fork the `ros2-package` branch, modify the config, and update the clone URL in the Dockerfile

### Multi-Sensor Setup

The driver supports up to 4 sensors. Each sensor can be:
- A **regular sensor** with external ROM file
- An **autoconfigured tool** (NDI pen) with internal ROM

#### Configuration Examples

**Single sensor with ROM:**
```yaml
num_sensors: 1
tool_rom_files: 
  - "/workspace/src/aurora_ndi_ros2_driver/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
port_handles: ["0A"]
port_handles_autoconfig: [""]
topic_names: ["/aurora_data_sensor0"]
child_frame_names: ["endo_aurora_sensor0"]
```

**Single NDI pen (autoconfigured):**
```yaml
num_sensors: 1
tool_rom_files: [""]  # Empty - pen has internal ROM
port_handles: ["0A"]
port_handles_autoconfig: ["0A"]
topic_names: ["/aurora_data_pen"]
child_frame_names: ["endo_aurora_pen"]
```

**Mixed: 1 sensor + 1 pen:**
```yaml
num_sensors: 2
tool_rom_files:
  - "/workspace/src/aurora_ndi_ros2_driver/rom/610029_AuroraMini6DOF_1.8x9mm/610029-6DOF.rom"
port_handles: ["0A", "0B"]
port_handles_autoconfig: ["0B"]
topic_names:
  - "/aurora_data_sensor0"
  - "/aurora_data_pen"
child_frame_names:
  - "endo_aurora_sensor0"
  - "endo_aurora_pen"
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | USB serial device |
| `num_sensors` | `1` | Total number of sensors (1-4) |
| `publish_rate_hz` | `40.0` | Publishing frequency |
| `enable_data_filtering` | `true` | Enable moving average filter |
| `debug_mode` | `false` | Enable verbose logging |

For complete parameter reference, see the [`ros2-package`](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/ros2-package) branch README.

## Usage

### Starting the System

**First time setup:**
Build the Docker image first:
```bash
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
```

Then start the container:
```bash
docker compose up
```

Start in detached mode (background):
```bash
docker compose up -d
```

### Accessing the Container

Open a shell inside the running container:
```bash
docker exec -it aurora_ndi_tracker bash
```

### ROS2 Commands

Inside the container, you can use all ROS2 commands:

**List topics:**
```bash
ros2 topic list
```

**Echo Aurora data:**
```bash
ros2 topic echo /aurora_data_sensor0
```

**Check TF transforms:**
```bash
ros2 run tf2_ros tf2_echo aurora_base endo_aurora_sensor0
```

**View TF tree:**
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

### Visualization

Launch RViz2 from inside the container:
```bash
ros2 run rviz2 rviz2
```

Launch rqt image view:
```bash
ros2 run rqt_image_view rqt_image_view
```

### Stopping the System

Stop the container:
```bash
docker compose down
```

Stop and remove all data:
```bash
docker compose down -v
```

## Troubleshooting

### Container Won't Start

**Error: "Cannot connect to the Docker daemon"**

Solution:
```bash
sudo systemctl start docker
sudo usermod -aG docker $USER
# Log out and back in
```

**Error: "device not found: /dev/ttyUSB0"**

Solution:
1. Check if device exists:
```bash
ls -l /dev/ttyUSB*
```

2. Ensure Aurora is connected and powered on

3. Update `docker-compose.yml` with correct device path:
```yaml
devices:
  - /dev/ttyUSB1:/dev/ttyUSB0  # Map USB1 to USB0 in container
```

### GUI Applications Not Working

**Error: "cannot open display"**

Solution:
```bash
# Allow Docker to access X server
xhost +local:root

# Or more secure - allow only specific container
xhost +local:docker
```

For permanent solution, add to `~/.bashrc`:
```bash
xhost +local:root
```

### Aurora Device Permissions

**Error: "Permission denied: /dev/ttyUSB0"**

Solution:
```bash
# On host machine
sudo chmod 666 /dev/ttyUSB0
```

Or add your user to dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Configuration Not Loading

**Problem:** Changes to config file not reflected

Solution:
The config file is inside the container. After editing it, restart the container:

```bash
docker compose restart
```

Or rebuild the image if you need the latest version from GitHub:
```bash
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
docker compose up
```

### Rebuild After Changes

**To get the latest driver code from GitHub:**

```bash
# Stop container
docker compose down

# Rebuild with cache busting (forces re-clone)
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .

# Start fresh
docker compose up
```

**For a complete clean rebuild:**

```bash
# Remove old image and rebuild from scratch
docker compose down
docker rmi aurora_ndi_tracker:latest
docker build --no-cache --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
docker compose up
```

### View Container Logs

See what's happening inside:
```bash
docker compose logs -f
```

View only Aurora node logs:
```bash
docker exec -it aurora_ndi_tracker bash
ros2 topic echo /rosout
```

## Advanced Usage

### Custom ROM Files

To use your own ROM files, mount a volume in `docker-compose.yml`:

```yaml
volumes:
  - ./rom:/workspace/custom_rom:ro
```

Then edit the config inside the container and reference your ROM files:
```bash
docker exec -it aurora_ndi_tracker bash
nano /workspace/src/aurora_ndi_ros2_driver/config/aurora_config.yaml
# Update tool_rom_files to point to /workspace/custom_rom/your_sensor.rom
```

### Recording Data

Mount a directory for ROS2 bags in `docker-compose.yml`:

```yaml
volumes:
  - ./bags:/workspace/bags
```

Inside container:
```bash
cd /workspace/bags
ros2 bag record /aurora_data_sensor0 /tf
```

### Development Workflow

For active development on the driver, you have two options:

**Option 1: Edit inside container**
```bash
docker exec -it aurora_ndi_tracker bash
cd /workspace/src/aurora_ndi_ros2_driver
# Make your changes
cd /workspace
colcon build --packages-select aurora_ndi_ros2_driver
source install/setup.bash
```

**Option 2: Mount local development copy**
1. Clone the `ros2-package` branch locally
2. Add volume mount in `docker-compose.yml`:
```yaml
volumes:
  - /path/to/local/aurora_ndi_ros2_driver:/workspace/src/aurora_ndi_ros2_driver
```
3. Rebuild inside container when you make changes:
```bash
docker exec -it aurora_ndi_tracker bash
cd /workspace
colcon build --packages-select aurora_ndi_ros2_driver
source install/setup.bash
```

### Getting Latest Updates

To pull the latest driver code from GitHub:

```bash
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
docker compose up
```

The `CACHEBUST` argument ensures Docker re-clones the repository even if other layers are cached.

### Using Different USB Device

If your Aurora is on a different USB port, edit `docker-compose.yml`:

```yaml
devices:
  - /dev/ttyUSB1:/dev/ttyUSB0  # Maps host USB1 to container USB0
```

Or edit the config inside the container:
```bash
docker exec -it aurora_ndi_tracker bash
nano /workspace/src/aurora_ndi_ros2_driver/config/aurora_config.yaml
# Change serial_port to "/dev/ttyUSB1"
```

### Multiple Containers

To run multiple Aurora systems simultaneously:

1. Copy this directory
2. Rename the service in `docker-compose.yml`:
```yaml
services:
  aurora_tracker_2:
    container_name: aurora_ndi_tracker_2
```

3. Map different USB devices and ports

## Contributing

Found a bug or want to improve the Docker setup?

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

Property of Edoardo Guida, student at Politecnico di Milano, developed at NEARlab

---

**Need the pure ROS2 package?** Check out the [`ros2-package`](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/ros2-package) branch for installation in your existing ROS2 workspace.