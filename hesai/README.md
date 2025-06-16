# Hesai LiDAR Driver for Cone Detection

A ROS package for interfacing with the Hesai Pandar 40p LiDAR sensor and detecting traffic cones for autonomous vehicle applications.

## Overview

This package provides:

- UDP communication with Hesai Pandar 40p LiDAR
- Real-time point cloud processing and filtering
- Cone detection using clustering and geometric analysis
- ROS integration with standard sensor_msgs and custom bristol_msgs

## Hardware Requirements

- Hesai Pandar 40p LiDAR sensor
- USB ethernet adaptor for sensor connection
- Computer with Ubuntu and ROS installed

## Dependencies

### System Dependencies

```bash
sudo apt-get install ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions
sudo apt-get install ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros
sudo apt-get install ros-${ROS_DISTRO}-dynamic-reconfigure
sudo apt-get install libboost-system-dev libboost-thread-dev
```

### Custom Dependencies

- `bristol_msgs` - For ConeArrayWithCovariance message type

## Installation

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <repository_url> hesai_lidar_driver
```

2. Build the package:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Network Configuration

The Hesai Pandar 40p typically uses:

- **Device IP**: 192.168.1.201
- **Data Port**: 2368 (UDP)
- **Position Port**: 8308 (UDP)

Ensure your ethernet interface is configured to communicate with the LiDAR:

```bash
sudo ip addr add 192.168.1.100/24 dev eth0  # Adjust interface name as needed
```

## Usage

### 🚗 Production Mode (Real-Time)
For autonomous vehicle applications and real-time processing:

```bash
# Method 1: Direct execution
ros2 run hesai_lidar_driver hesai_lidar_node

# Method 2: Launch file with RViz
ros2 launch hesai_lidar_driver hesai_visualization.launch.py
```

**Features**: Zero latency, real-time processing, production-ready

### 🗺️ Demonstration Mode (Mapping Visualisation)
For mapping demonstrations and presentations:

```bash
# Method 1: Direct execution  
ros2 run hesai_lidar_driver hesai_demo_node

# Method 2: Launch file with demo RViz config
ros2 launch hesai_lidar_driver hesai_mapping_demo.launch.py

# Method 3: Convenience script
./scripts/launch_mapping_demo.sh
```

**⚠️ WARNING: DEMONSTRATION ONLY - NOT FOR REAL-TIME USE**

**Features**: 60-second point persistence, accumulated mapping view, timing warnings

### Custom Parameters

```bash
ros2 launch hesai_lidar_driver hesai_visualization.launch.py device_ip:=192.168.1.201
```

## Configuration

### Launch File Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device_ip` | 192.168.1.201 | LiDAR sensor IP address |
| `data_port` | 2368 | UDP port for data packets |
| `position_port` | 8308 | UDP port for position packets |
| `frame_id` | hesai_lidar | Frame ID for point cloud |
| `launch_rviz` | true | Launch RViz visualisation |

### Processing Parameters

- **Region of Interest**: Define X/Y/Z boundaries for processing
- **Filtering**: Voxel grid downsampling, statistical outlier removal
- **Ground Removal**: Height-based ground point filtering
- **Cone Detection**: Clustering and geometric validation parameters

### Dynamic Reconfigure

Real-time parameter adjustment is available via:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/hesai_lidar/raw_cloud` | sensor_msgs/PointCloud2 | Raw LiDAR point cloud |
| `/hesai_lidar/filtered_cloud` | sensor_msgs/PointCloud2 | Processed point cloud |
| `/hesai_lidar/detected_cones` | bristol_msgs/ConeArrayWithCovariance | Detected cone positions |
| `/hesai_lidar/cone_markers` | visualization_msgs/MarkerArray | Cone visualisation markers |

### Subscribed Topics

None (driver generates data from hardware)

## Frame Conventions

- **Coordinate System**: ROS standard (X forward, Y left, Z up)
- **Base Frame**: `base_link`
- **LiDAR Frame**: `hesai_lidar`

## Visualisation

Launch with RViz to visualise:

- Raw and processed point clouds
- Detected cone positions and markers
- LiDAR coordinate frames

Custom RViz configuration is provided in `rviz/hesai_cone_detection.rviz`.

## Troubleshooting

### Common Issues

1. **No data received**:
   - Check network configuration and IP settings
   - Verify LiDAR power and ethernet connection
   - Check firewall settings for UDP ports

2. **Poor cone detection**:
   - Adjust clustering parameters in launch file
   - Tune cone geometry constraints
   - Verify ground removal settings

3. **High CPU usage**:
   - Increase voxel grid leaf size for downsampling
   - Reduce point cloud processing region
   - Adjust statistical outlier removal parameters

### Debug Topics

Enable debug publishing by setting parameters:

- `/hesai_lidar/publish_debug_clouds` - Intermediate processing stages
- `/hesai_lidar/publish_cluster_clouds` - Individual cone clusters

## Development

### Project Structure

```text
hesai_lidar_driver/
├── include/hesai_lidar_driver/    # Header files
├── src/                           # Source files
├── launch/                        # ROS launch files
├── config/                        # Configuration files
├── rviz/                         # RViz configurations
├── test/                         # Unit tests
└── scripts/                      # Utility scripts
```

### Building from Source

Ensure all dependencies are installed, then:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps hesai_lidar_driver
```

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with appropriate tests
4. Submit a pull request

## Authors

- Seth - Initial development

## Acknowledgements

- Hesai Technology for LiDAR sensor documentation
- UWE Bristol AI team for bristol_msgs package
- ROS community for foundational libraries
