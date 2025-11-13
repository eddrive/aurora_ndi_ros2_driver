# Aurora NDI ROS2 Driver - Docker Setup

Containerized ROS2 Humble environment for NDI Aurora electromagnetic tracking.

## Overview

**What's included:**
- ROS2 Humble + Aurora driver (auto-cloned from `ros2-package` branch)
- GUI support (RViz2, rqt)
- USB device mapping and X11 forwarding
- Ready-to-run container

**Branches:**
- **`docker-setup`** (current): Containerized deployment
- **`ros2-package`**: Pure ROS2 package for existing workspaces

Switch to ROS2 package: `git checkout ros2-package`

## Prerequisites

- Docker 20.10+ & Docker Compose 2.0+
- NDI Aurora system with USB connection (`/dev/ttyUSB0`)

**Install Docker:**
```bash
curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh
sudo usermod -aG docker $USER
sudo apt-get install docker-compose-plugin
# Log out and back in
```

## Quick Start

```bash
# 1. Clone and enter directory
git clone -b docker-setup https://github.com/eddrive/aurora_ndi_ros2_driver.git
cd aurora_ndi_ros2_driver

# 2. Enable X11 for GUI (RViz2)
xhost +local:root

# 3. Build (with latest driver from ros2-package branch)
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .

# 4. Start
docker compose up
```

**Access container:**
```bash
docker exec -it aurora_ndi_tracker bash
```

## Configuration

**Location:** `/workspace/src/aurora_ndi_ros2_driver/config/aurora_config.yaml`

**Edit config:**
```bash
docker exec -it aurora_ndi_tracker bash
nano /workspace/src/aurora_ndi_ros2_driver/config/aurora_config.yaml
docker compose restart  # Apply changes
```

### Multi-Sensor Examples

```yaml
# Single sensor
num_sensors: 1
tool_rom_files: ["/workspace/src/.../sensor.rom"]
port_handles: ["0A"]
port_handles_autoconfig: [""]

# Single NDI pen (autoconfigured)
num_sensors: 1
tool_rom_files: [""]
port_handles: ["0A"]
port_handles_autoconfig: ["0A"]

# Mixed (1 sensor + 1 pen)
num_sensors: 2
tool_rom_files: ["/workspace/src/.../sensor.rom"]
port_handles: ["0A", "0B"]
port_handles_autoconfig: ["0B"]
```

Full config reference: [`ros2-package` README](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/ros2-package)

## Usage

**Start/Stop:**
```bash
docker compose up           # Start (foreground)
docker compose up -d        # Start (background)
docker compose down         # Stop
docker compose logs -f      # View logs
```

**ROS2 Commands (inside container):**
```bash
docker exec -it aurora_ndi_tracker bash

# Monitor topics
ros2 topic list
ros2 topic echo /aurora_data_sensor0
ros2 topic hz /aurora_data_sensor0

# TF transforms
ros2 run tf2_ros tf2_echo aurora_base endo_aurora_sensor0
ros2 run rqt_tf_tree rqt_tf_tree

# Visualization
ros2 run rviz2 rviz2
```

**Data Recording:**
```bash
# Add to docker-compose.yml:
# volumes:
#   - ./bags:/workspace/bags

docker exec -it aurora_ndi_tracker bash
cd /workspace/bags
ros2 bag record /aurora_data_sensor0 /tf
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **Docker daemon not running** | `sudo systemctl start docker` |
| **Device not found** | Check `ls -l /dev/ttyUSB*`, update `docker-compose.yml` device path |
| **Permission denied** | `sudo chmod 666 /dev/ttyUSB0` or `sudo usermod -aG dialout $USER` |
| **GUI not working** | `xhost +local:root` (add to `~/.bashrc` for persistence) |
| **Config not loading** | Edit in container, then `docker compose restart` |

**Rebuild with latest code:**
```bash
docker compose down
docker build --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
docker compose up
```

**Clean rebuild:**
```bash
docker compose down
docker rmi aurora_ndi_tracker:latest
docker build --no-cache --build-arg CACHEBUST=$(date +%s) -t aurora_ndi_tracker:latest .
docker compose up
```

## Advanced Usage

### Custom ROM Files
```yaml
# docker-compose.yml
volumes:
  - ./rom:/workspace/custom_rom:ro
```

### Development Mode
```yaml
# docker-compose.yml - mount local code
volumes:
  - /path/to/local/aurora_ndi_ros2_driver:/workspace/src/aurora_ndi_ros2_driver
```

```bash
# Rebuild after changes
docker exec -it aurora_ndi_tracker bash
cd /workspace && colcon build --packages-select aurora_ndi_ros2_driver
source install/setup.bash
```

### Different USB Device
```yaml
# docker-compose.yml
devices:
  - /dev/ttyUSB1:/dev/ttyUSB0
```

### Multiple Containers
Duplicate directory and update `docker-compose.yml`:
```yaml
services:
  aurora_tracker_2:
    container_name: aurora_ndi_tracker_2
```

## License & Info

**Copyright © 2024 Edoardo Guida**
Developed at [NEARlab, Politecnico di Milano](https://nearlab.polimi.it/)
Contact: [edoardo1.guida@mail.polimi.it](mailto:edoardo1.guida@mail.polimi.it)

---

**ROS2 package branch**: [`ros2-package`](https://github.com/eddrive/aurora_ndi_ros2_driver/tree/ros2-package)