# Velodyne Point Cloud Troubleshooting Guide

## Issue: Velodyne point clouds not appearing on map in RViz

This guide will help you systematically debug why Velodyne LiDAR point clouds aren't showing up in RViz when running Autoware on your shuttle bus.

## Your Configuration (from analysis)

Based on your `nuway_sensor_kit_launch` configuration:

- **Front Velodyne VLP16**:
  - IP: `192.168.5.27`
  - UDP Port: `2369`
  - Frame: `lidar_velodyne_front`
  - Position: 1.6m forward, 0.9m up from sensor_kit_base_link

- **Rear Velodyne VLP16**:
  - IP: `192.168.5.28`
  - UDP Port: `2368`
  - Frame: `lidar_velodyne_rear`
  - Position: -1.6m rear, 0.9m up from sensor_kit_base_link

- **Host IP**: `192.168.5.30`

- **Driver**: Nebula (Universal LiDAR driver framework)

## Step-by-Step Debugging

### 1. Network Connectivity Check

First, verify network connectivity to the Velodyne sensors:

```bash
# Ping front LiDAR
ping -c 3 192.168.5.27

# Ping rear LiDAR
ping -c 3 192.168.5.28

# Check if UDP packets are being received on the data ports
sudo tcpdump -i any udp port 2369 -c 10  # Front LiDAR
sudo tcpdump -i any udp port 2368 -c 10  # Rear LiDAR
```

**Expected**: You should see successful pings and UDP packets flowing.

**If failing**:
- Check physical network cable connections
- Verify your computer's network interface IP is `192.168.5.30` (or adjust `host_ip` in launch file)
- Ensure firewall isn't blocking UDP ports 2368-2369
- Check Velodyne is powered on (look for spinning/status lights)

### 2. Check Network Interface Configuration

```bash
# List network interfaces
ip addr show

# Check routing table
ip route show

# Verify interface is on correct subnet (192.168.5.x)
```

**Fix if needed**:
```bash
# Set static IP on your Ethernet interface (replace eth0 with your interface)
sudo ip addr add 192.168.5.30/24 dev eth0
sudo ip link set eth0 up

# Or configure via netplan (Ubuntu):
sudo nano /etc/netplan/01-network-manager-all.yaml
```

### 3. Verify ROS 2 Environment is Running

After launching Autoware, check if nodes are running:

```bash
# List all ROS 2 nodes
ros2 node list

# Look for Velodyne/Nebula nodes - you should see something like:
# /sensing/lidar/lidar_velodyne_front/nebula_ros_node
# /sensing/lidar/lidar_velodyne_rear/nebula_ros_node
```

**If nodes are missing**:
- Check Autoware launch logs for errors
- Verify `launch_driver` argument is `true` in your launch command

### 4. Check Point Cloud Topics

Verify that point cloud data is being published:

```bash
# List all topics
ros2 topic list

# Look for point cloud topics:
# /sensing/lidar/lidar_velodyne_front/pointcloud
# /sensing/lidar/lidar_velodyne_rear/pointcloud
# /sensing/lidar/concatenated/pointcloud

# Check if data is being published on front LiDAR
ros2 topic echo /sensing/lidar/lidar_velodyne_front/pointcloud --once

# Check message rate (should be ~10Hz for VLP16)
ros2 topic hz /sensing/lidar/lidar_velodyne_front/pointcloud

# Check bandwidth
ros2 topic bw /sensing/lidar/lidar_velodyne_front/pointcloud
```

**Expected**:
- Topics should exist
- Data rate: ~10 Hz
- Bandwidth: Several MB/s

**If no data**:
- Check driver logs: `ros2 node info /sensing/lidar/lidar_velodyne_front/nebula_ros_node`
- Verify network connectivity (Step 1)
- Check if Velodyne is configured correctly (web interface at http://192.168.5.27)

### 5. Verify TF (Transform) Tree

Point clouds need proper transforms to be visualized:

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# This creates frames.pdf - open it to verify transform chain:
# map → base_link → sensor_kit_base_link → lidar_velodyne_front
# map → base_link → sensor_kit_base_link → lidar_velodyne_rear

# Check specific transform
ros2 run tf2_ros tf2_echo base_link lidar_velodyne_front

# List all frames
ros2 run tf2_tools tf2_monitor
```

**Expected**: You should see a complete transform chain from `map` to your LiDAR frames.

**If transforms are missing**:
- Check if `robot_state_publisher` or `static_transform_publisher` nodes are running
- Verify sensor calibration files are being loaded
- Check localization node is running (provides map → base_link)

### 6. Check RViz Configuration

Common RViz configuration issues:

```bash
# Launch RViz separately to debug
rviz2
```

**In RViz**:

1. **Fixed Frame**: Check the "Global Options" → "Fixed Frame"
   - Should be `map` (if localization is running)
   - Or `base_link` (if just testing sensors)
   - Try changing to `lidar_velodyne_front` temporarily to see if cloud appears

2. **Add PointCloud2 Display**:
   - Click "Add" → "By topic" → Select `/sensing/lidar/lidar_velodyne_front/pointcloud`
   - Or "By display type" → `PointCloud2` then manually set topic

3. **PointCloud2 Display Settings**:
   - Size (Pixels): 3-5
   - Style: Points
   - Color Transformer: Try different options:
     - `Intensity` - uses LiDAR intensity values
     - `AxisColor` - colors by X/Y/Z axis
     - `FlatColor` - single color
   - Decay Time: 0 (or increase to accumulate points)

4. **Check Topic Status**:
   - Look at the bottom left of RViz
   - Topic should show "OK" not "No messages received"

5. **Check View Angle**:
   - Use mouse to rotate camera
   - Point cloud might be behind you or very far away
   - Try "Reset" view or use "Orbit" camera

### 7. Check Autoware System Status

```bash
# Check if all sensing components are healthy
ros2 topic echo /diagnostics

# Look for Velodyne/Nebula diagnostics
# Status should be "OK" not "ERROR" or "WARN"

# Check Velodyne monitor status
ros2 topic echo /diagnostics_agg

# Check specific namespace
ros2 topic echo /sensing/lidar/lidar_velodyne_front/diagnostics
```

### 8. Review Launch Configuration

Verify launch parameters are correct:

**File**: `/home/user/autoware.shuttle_bus/src/launcher/autoware_launch/sensor_kit/nuway_sensor_kit_launch/nuway_sensor_kit_launch/launch/lidar.launch.xml`

Key parameters to verify:
- `sensor_ip`: Matches your Velodyne IPs (192.168.5.27/28)
- `host_ip`: Matches your computer's IP (192.168.5.30)
- `data_port`: 2369 (front), 2368 (rear)
- `launch_driver`: Set to `true`
- `sensor_frame`: `lidar_velodyne_front`, `lidar_velodyne_rear`

### 9. Check Nebula Driver Logs

```bash
# View logs from Nebula driver node
ros2 node info /sensing/lidar/lidar_velodyne_front/nebula_ros_node

# Check for error messages in logs
ros2 log echo /sensing/lidar/lidar_velodyne_front/nebula_ros_node

# Or check terminal output where you launched Autoware
# Look for errors like:
# - "Failed to connect to sensor"
# - "No packets received"
# - "Invalid calibration file"
```

### 10. Test with Minimal Setup

Try launching just the LiDAR driver without full Autoware:

```bash
# Source workspace
source /home/user/autoware.shuttle_bus/install/setup.bash

# Launch just the front Velodyne
ros2 launch nuway_sensor_kit_launch sensing.launch.xml \
  sensor_model:=nuway_sensor_kit \
  vehicle_id:=default \
  launch_driver:=true

# In another terminal, check topics
ros2 topic list
ros2 topic echo /sensing/lidar/lidar_velodyne_front/pointcloud --once
```

### 11. Verify Velodyne Sensor Configuration

Access Velodyne web interface:

```bash
# Open in browser
http://192.168.5.27  # Front LiDAR
http://192.168.5.28  # Rear LiDAR
```

**Check**:
- Network settings (IP, gateway, subnet mask)
- Data port configuration (should match launch file)
- Returns mode (Single/Dual/Strongest/Last)
- Motor RPM (should be 600 for VLP16)
- Laser enable status

## Common Issues and Solutions

### Issue 1: "No messages received" in RViz

**Cause**: Topic name mismatch or no data being published

**Solution**:
```bash
# Verify exact topic name
ros2 topic list | grep pointcloud

# Use that exact name in RViz
# Common topics:
# - /sensing/lidar/lidar_velodyne_front/pointcloud (raw)
# - /sensing/lidar/concatenated/pointcloud (merged)
# - /sensing/lidar/top/outlier_filtered/pointcloud (processed)
```

### Issue 2: "Transform [sender=unknown_publisher] ... not available"

**Cause**: Missing transform between fixed frame and point cloud frame

**Solution**:
```bash
# Check available transforms
ros2 run tf2_tools view_frames

# Change RViz fixed frame to match an available frame
# Try: base_link, sensor_kit_base_link, or lidar_velodyne_front
```

### Issue 3: Point cloud appears but not on map

**Cause**: Localization not running or map not loaded

**Solution**:
- Ensure you've loaded a map
- Check localization node is running: `ros2 node list | grep localiz`
- Verify initial pose is set in RViz (use "2D Pose Estimate" tool)
- Check `/tf` topic has `map → base_link` transform

### Issue 4: No UDP packets received

**Cause**: Network configuration or firewall issues

**Solution**:
```bash
# Check firewall rules
sudo ufw status

# Allow UDP ports if blocked
sudo ufw allow 2368:2369/udp

# Or disable firewall temporarily for testing
sudo ufw disable

# Check if another process is using the port
sudo netstat -tulpn | grep 2368
```

### Issue 5: Points appear but are all in one location

**Cause**: Incorrect sensor calibration or transform issues

**Solution**:
- Verify calibration file: `/home/user/autoware.shuttle_bus/src/launcher/autoware_launch/sensor_kit/nuway_sensor_kit_launch/nuway_sensor_kit_description/config/sensor_kit_calibration.yaml`
- Check transforms are being published correctly
- Ensure `base_link` frame is moving (if vehicle is stationary, localization might not update)

### Issue 6: Nebula driver fails to start

**Cause**: Missing dependencies or incompatible Velodyne model

**Solution**:
```bash
# Check Nebula supports your Velodyne model (VLP16 is supported)
# Verify dependencies are installed
rosdep install -y --from-paths src --ignore-src --rosdistro humble

# Check launch file for typos in model name
# Should be: model="VLP16" not "VLP-16" or "vlp16"
```

## Quick Diagnostic Command Summary

Run these commands in sequence and note any errors:

```bash
# 1. Network
ping -c 3 192.168.5.27
sudo tcpdump -i any udp port 2369 -c 5

# 2. ROS nodes
ros2 node list | grep velodyne

# 3. Topics
ros2 topic list | grep pointcloud
ros2 topic hz /sensing/lidar/lidar_velodyne_front/pointcloud

# 4. Transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link lidar_velodyne_front

# 5. Diagnostics
ros2 topic echo /diagnostics --once

# 6. View in RViz
rviz2
```

## Expected Working State

When everything is working correctly, you should see:

1. ✅ Successful ping to both Velodyne sensors
2. ✅ UDP packets flowing on ports 2368 and 2369
3. ✅ Nebula driver nodes running
4. ✅ Point cloud topics publishing at ~10 Hz
5. ✅ Complete TF tree from map → base_link → sensor_kit_base_link → lidar frames
6. ✅ No errors in `/diagnostics`
7. ✅ Point clouds visible in RViz with proper colors and structure

## Additional Resources

- **Nebula Documentation**: https://github.com/tier4/nebula
- **Velodyne VLP16 Manual**: Check manufacturer documentation
- **Autoware Sensor Configuration**: `/home/user/autoware.shuttle_bus/src/launcher/autoware_launch/sensor_kit/nuway_sensor_kit_launch/`
- **ROS 2 TF2 Debugging**: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html

## Contact Information

If you've tried all these steps and still have issues, collect the following information:

1. Output of `ros2 node list`
2. Output of `ros2 topic list`
3. Output of `ros2 topic hz /sensing/lidar/lidar_velodyne_front/pointcloud`
4. Screenshot of `view_frames` output (frames.pdf)
5. Any error messages from Autoware launch
6. RViz screenshot showing your configuration
7. Network configuration: `ip addr show`

---

**Last Updated**: 2025-11-16
**Configuration**: nuway_sensor_kit with dual Velodyne VLP16
**Driver**: Nebula (via common_sensor_launch)
