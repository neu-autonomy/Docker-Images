# DLIOM with Ouster LiDAR Setup

This repository contains the setup for Direct LiDAR-Inertial Odometry and Mapping (DLIOM) using an Ouster LiDAR sensor with ROS2.

## Prerequisites

- Docker installed on your system
- Ouster LiDAR connected via Ethernet
- X11 forwarding support for GUI applications
- The `dliom-feature-ros2` package downloaded to the same folder as the Docker images

## Hardware Setup

1. **Connect the Ouster LiDAR** via Ethernet cable
2. **Configure Network Settings**: 
   - Ensure the Ethernet connection is enabled in your network settings
   - Set the connection to use a local link
3. **Wait for Hostname Assignment**: Allow 60 seconds for the host name to be automatically assigned

## Quick Start (Automated)

Use the provided automation script for a one-command setup:

```bash
./run_dliom.sh
```

This script will automatically:
- Detect your Ouster LiDAR IP address
- Build the necessary Docker images
- Start all required containers and ROS2 nodes
- Launch DLIOM with RViz visualization

## Manual Setup

If you prefer to run the setup manually or need to troubleshoot:

### Step 1: Build and Test Ouster Connection

```bash
# Build the DLIOM Docker image
docker build -f Dockerfile.ouster -t ouster:latest .

# Test connection to Ouster (replace with your device's hostname)
ping -4 -c3 os-1222225000331.local

# Note the IP address from the ping response
# Enable X11 forwarding
xhost +local:docker
```

### Step 2: Start Ouster Driver Container

```bash
docker run -it --rm \
  --network=host \
  --privileged \
  -v /dev:/dev \
  -v /sys:/sys \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ouster:latest
```

### Step 3: Set Up Topic Relays

In separate terminals, run these commands to create best-effort topic relays:

**Terminal 2 - IMU Relay:**
```bash
ros2 run topic_tools relay /ouster/imu /ouster/imu_best_effort \
  --qos-reliability best_effort --qos-durability volatile
```

**Terminal 3 - Point Cloud Relay:**
```bash
ros2 run topic_tools relay /ouster/points /ouster/points_best_effort \
  --qos-reliability best_effort --qos-durability volatile
```

### Step 4: Launch DLIOM

**Terminal 4:**
```bash
# Build DLIOM image (if not already built)
docker build -f Dockerfile.dliom -t dliom:latest .

# Enable X11 forwarding
xhost +local:docker

# Run DLIOM container
docker run -it --rm \
  --network=host \
  --privileged \
  -v /dev:/dev \
  -v /sys:/sys \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  dliom:latest

# Inside the container, launch DLIOM
ros2 launch direct_lidar_inertial_odometry_and_mapping dliom.launch.py \
  rviz:=true \
  pointcloud_topic:=/ouster/points \
  imu_topic:=/ouster/imu
```

## File Structure

```
project/
├── dliom-feature-ros2/          # Required ROS2 package
├── Dockerfile.dliom             # DLIOM Docker configuration
├── docker-compose.yml           # Docker Compose configuration
├── run_dliom.sh                # Automated setup script
└── README.md                   # This file
```

## Troubleshooting

### Common Issues

1. **LiDAR Not Detected**: 
   - Verify Ethernet connection is active
   - Check if hostname follows format: `os-[serial_number].local`
   - Wait full 60 seconds for hostname assignment

2. **Docker Permission Issues**:
   - Ensure user is in docker group: `sudo usermod -aG docker $USER`
   - Restart terminal session after adding to docker group

3. **X11 Display Issues**:
   - Run `xhost +local:docker` before starting containers
   - Verify `$DISPLAY` environment variable is set

4. **Topic Connection Issues**:
   - Check if topics are being published: `ros2 topic list`
   - Verify QoS settings match between publisher and subscriber

### Debugging Commands

```bash
# List available topics
ros2 topic list

# Check topic information
ros2 topic info /ouster/points
ros2 topic info /ouster/imu

# Monitor topic data
ros2 topic echo /ouster/points --once
ros2 topic echo /ouster/imu --once

# Check Docker containers
docker ps -a

# View container logs
docker logs <container_id>
```

## Configuration

### Ouster LiDAR Settings

The Ouster hostname format is: `os-[serial_number].local`
- Serial number can be found on the device sticker
- Example: `os-1222225000331.local`

### DLIOM Parameters

Key parameters can be modified in the launch command:
- `rviz:=true/false` - Enable/disable RViz visualization
- `pointcloud_topic:=/ouster/points` - Point cloud topic name
- `imu_topic:=/ouster/imu` - IMU topic name

## Support

For issues specific to:
- **DLIOM**: Check the original DLIOM documentation
- **Ouster LiDAR**: Refer to Ouster SDK documentation
- **ROS2**: Consult ROS2 documentation and forums

## License

Please refer to the individual component licenses for DLIOM and Ouster SDK.
