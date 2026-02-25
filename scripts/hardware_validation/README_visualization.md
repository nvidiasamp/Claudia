# LiDAR Point Cloud Visualization Guide for SSH Environments

> **Applicable scenario**: Remote SSH connection where graphical interfaces are not directly accessible
> **Goal**: View Unitree Go2 4D LiDAR L1 point cloud data
> **Topic**: `/utlidar/cloud`

---

## Quick Start

### Environment Setup
```bash
# Enter the project directory
cd ~/claudia

# Load ROS2 environment
source /opt/ros/foxy/setup.bash
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Verify point cloud topic is available
ros2 topic list | grep utlidar
ros2 topic info /utlidar/cloud
```

---

## Option Selection

### **Option 1: X11 Forwarding (Best User Experience)**
```bash
# Reconnect SSH with X11 forwarding enabled
ssh -X username@robot_ip
# Or
ssh -Y username@robot_ip  # Trusted X11 forwarding

# Run RViz2 viewer
./scripts/hardware_validation/rviz_pointcloud_viewer.sh
```

**RViz2 Configuration Steps**:
1. Add `PointCloud2` display type
2. Set Topic to: `/utlidar/cloud`
3. Set Fixed Frame to: `utlidar_lidar`
4. Adjust point cloud size and color scheme

**Pros**: Real-time interaction, 3D manipulation
**Cons**: Requires X11 support, high bandwidth requirements

---

### **Option 2: Generate Static Images (Recommended for SSH Users)**
```bash
# Generate multi-view point cloud images
python3 scripts/hardware_validation/static_pointcloud_viewer.py
```

**Output**:
- `logs/pointcloud_images/` directory
- 5 high-resolution PNG images
- Includes statistics and multi-view visualizations

**Features**:
- Top view, side view, front view
- Distance distribution histogram
- Detailed statistics
- 300 DPI high-quality output

**Pros**: No graphical interface needed, small file size, easy to share
**Cons**: Static images, no interaction

---

### **Option 3: Save PCD Files (Professional Analysis)**
```bash
# Save standard PCD format files
python3 scripts/hardware_validation/save_pointcloud_pcd.py
```

**Output**:
- `logs/pointcloud_pcd/` directory
- 3 standard PCD format files
- Contains complete point cloud data

**Local Viewing Tools**:
```bash
# CloudCompare (recommended)
# Download: https://www.cloudcompare.org/

# PCL Viewer
pcl_viewer filename.pcd

# MeshLab
# Download: https://www.meshlab.net/

# Python Open3D
python3 -c "import open3d as o3d; pcd = o3d.io.read_point_cloud('filename.pcd'); o3d.visualization.draw_geometries([pcd])"
```

**Pros**: Standard format, professional tool support, complete data
**Cons**: Requires download, local software needed

---

### **Option 4: Real-time Data Monitoring**
```bash
# View real-time point cloud topic statistics
ros2 topic hz /utlidar/cloud
ros2 topic bw /utlidar/cloud

# View point cloud message structure
ros2 topic echo --no-arr /utlidar/cloud | head -20
```

**Output Information**:
- Publish frequency (Hz)
- Bandwidth usage (MB/s)
- Message structure and fields
- Frame ID information

---

## Troubleshooting

### **Cannot connect to topic**
```bash
# Check ROS2 environment
echo $RMW_IMPLEMENTATION  # Should display: rmw_cyclonedds_cpp
ros2 daemon stop && ros2 daemon start

# Check topic status
ros2 topic list | grep -i lidar
ros2 node list | grep -i lidar
```

### **Python script errors**
```bash
# Check dependencies
python3 -c "import rclpy, numpy, matplotlib; print('Dependencies OK')"

# Reinstall necessary packages
pip3 install --upgrade numpy matplotlib open3d
```

### **X11 forwarding failure**
```bash
# Test X11
echo $DISPLAY
xeyes  # Test program

# Local SSH configuration (~/.ssh/config)
Host robot
    ForwardX11 yes
    ForwardX11Trusted yes
```

---

## Performance Benchmarks

| Option | File Size | Generation Time | Bandwidth Required | Use Case |
|--------|-----------|-----------------|-------------------|----------|
| X11 Forwarding | N/A | Real-time | High (>1MB/s) | Real-time debugging |
| Static Images | ~2MB/image | 10-30s | Low | Reports/presentations |
| PCD Files | ~1-5MB/file | 5-15s | Low | Professional analysis |
| Data Monitoring | N/A | Real-time | Very low | System diagnostics |

---

## Best Practices

### **Development Phase**
1. Use Option 4 to monitor data stream status
2. Use Option 2 to generate static images for verification
3. Use Option 3 to save critical data when needed

### **Debugging Phase**
1. Try Option 1 first (X11 forwarding)
2. Fall back to Option 2 for multi-angle views
3. Use Option 3 to save problematic data

### **Deployment Phase**
1. Use Option 4 for continuous monitoring
2. Periodically use Option 2 to generate report images
3. Use Option 3 to save data at critical checkpoints

---

## Output File Organization

```
logs/
├── pointcloud_images/          # Static images
│   ├── pointcloud_frame_001_20250627_101530.png
│   ├── pointcloud_frame_002_20250627_101535.png
│   └── ...
├── pointcloud_pcd/            # PCD files
│   ├── unitree_go2_lidar_20250627_101530_001.pcd
│   ├── unitree_go2_lidar_20250627_101535_002.pcd
│   └── ...
└── lidar_l1_validation_*.json # Validation reports
```

---

## Support Information

- **Project documentation**: `docs/`
- **Validation report**: `logs/lidar_l1_validation_summary.md`
- **Technical support**: See TaskMaster task 4.1 details

**Script locations**:
- `scripts/hardware_validation/rviz_pointcloud_viewer.sh`
- `scripts/hardware_validation/static_pointcloud_viewer.py`
- `scripts/hardware_validation/save_pointcloud_pcd.py`
