# IMU Troubleshooting Guide

## Issue: IMU not providing orientation/angular velocity data

This guide will help you systematically debug IMU (Inertial Measurement Unit) issues on your shuttle bus running Autoware.

## Your Configuration (from analysis)

Based on your `nuway_sensor_kit_launch` configuration:

- **IMU Type**: Tamagawa IMU (default configuration)
- **Raw Data Topic**: `/imu/tamagawa/imu_raw`
- **Corrected Data Topic**: `/imu/imu_data`
- **Processing Pipeline**:
  1. **IMU Driver** → Raw IMU data (angular velocity, linear acceleration)
  2. **IMU Corrector** → Bias correction, coordinate transformation
  3. **Gyro Bias Estimator** → Continuous bias estimation from vehicle motion

- **Frame**: `imu_link` (from sensor calibration)
- **Position** (from calibration):
  - X: 0.0m, Y: 0.0m, Z: 2.1m (2.1m above sensor_kit_base_link)
  - Roll: 180°, Pitch: 0°, Yaw: 180° (mounted upside down)

## IMU Basics

**IMU Measurements**:
- **Angular Velocity** (gyroscope): Rotation rate around X, Y, Z axes (rad/s)
- **Linear Acceleration** (accelerometer): Acceleration along X, Y, Z axes (m/s²)
- **(Optional) Orientation**: Some IMUs provide fused orientation (quaternion)

**Coordinate Frames**:
- **IMU Frame**: Raw sensor frame
- **Base Link Frame**: Vehicle-fixed frame (corrected by imu_corrector)
- Transforms handle mounting orientation (e.g., upside down)

**Expected Values (stationary vehicle)**:
- Angular velocity: ~0 rad/s (all axes)
- Linear acceleration:
  - Z-axis: ~9.81 m/s² (gravity)
  - X, Y axes: ~0 m/s²

## Step-by-Step Debugging

### 1. Hardware Connection Check

Verify IMU physical connection:

```bash
# Check if IMU is connected via serial
ls /dev/imu
# Or search for IMU device
ls /dev/tty* | grep -E "(USB|ACM|imu)"

# Common devices:
# /dev/imu (symlink configured in udev)
# /dev/ttyUSB0, /dev/ttyACM0 (direct connection)

# Check permissions
ls -l /dev/imu

# Fix permissions if needed
sudo chmod 666 /dev/imu
# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout/login
```

**Expected**: Device exists and is readable/writable

**Physical checks**:
- IMU is securely mounted to vehicle
- No loose connections
- IMU is powered (if external power required)
- Mounting orientation matches calibration file

### 2. Check ROS 2 IMU Nodes

After launching Autoware:

```bash
# List all nodes - look for IMU nodes
ros2 node list | grep imu

# Expected nodes:
# /sensing/imu/imu_corrector
# /sensing/imu/gyro_bias_estimator
# (Optional) /sensing/imu/tamagawa/tag_serial_driver

# Check node info
ros2 node info /sensing/imu/imu_corrector
ros2 node info /sensing/imu/gyro_bias_estimator
```

**If nodes are missing**:
- Check `launch_driver` is set to `true`
- Verify IMU driver is enabled in launch file
- Check device permissions

### 3. Verify IMU Topics

Check if IMU data is being published:

```bash
# List all IMU topics
ros2 topic list | grep imu

# Expected topics:
# /sensing/imu/tamagawa/imu_raw (raw data from driver)
# /sensing/imu/imu_data (corrected data - MAIN OUTPUT)
# /sensing/imu/gyro_bias_estimator/gyro_bias (estimated bias)

# Check raw IMU data
ros2 topic echo /sensing/imu/tamagawa/imu_raw --once

# Expected output:
# header:
#   stamp: {sec: X, nanosec: Y}
#   frame_id: tamagawa/imu_link
# orientation: {x: 0, y: 0, z: 0, w: 1}  # May be invalid (-1 in covariance)
# angular_velocity: {x: 0.01, y: -0.02, z: 0.00}  # rad/s
# linear_acceleration: {x: 0.1, y: 0.2, z: 9.81}  # m/s²

# Check message rate (should be 50-200 Hz typical)
ros2 topic hz /sensing/imu/tamagawa/imu_raw
ros2 topic hz /sensing/imu/imu_data

# Check corrected IMU data
ros2 topic echo /sensing/imu/imu_data --once
```

### 4. Check IMU Data Quality

Monitor IMU measurements for sanity:

```bash
# Watch raw data in real-time
ros2 topic echo /sensing/imu/tamagawa/imu_raw

# For stationary vehicle, check:
# 1. Angular velocity should be near 0 (within ±0.1 rad/s)
# 2. Linear acceleration Z should be ~9.8 m/s² (gravity)
# 3. Linear acceleration X, Y should be small (~0 for level ground)

# For moving vehicle:
# - Angular velocity changes when turning
# - Linear acceleration changes during acceleration/braking
```

**Warning Signs**:
- All zeros (no data)
- Constant non-zero angular velocity (stuck values)
- Linear acceleration Z not near 9.8 m/s² (wrong orientation or bad sensor)
- Erratic, noisy values (interference, bad sensor)
- No changes when vehicle moves (frozen data)

### 5. Verify IMU Corrector Operation

Check if bias correction is working:

```bash
# Check corrector parameters
ros2 param list /sensing/imu/imu_corrector

# Get bias values
ros2 param get /sensing/imu/imu_corrector angular_velocity_offset_x
ros2 param get /sensing/imu/imu_corrector angular_velocity_offset_y
ros2 param get /sensing/imu/imu_corrector angular_velocity_offset_z

# These values should be updated by gyro_bias_estimator
# or set manually in config file

# Compare raw vs corrected
ros2 topic echo /sensing/imu/tamagawa/imu_raw --field angular_velocity
ros2 topic echo /sensing/imu/imu_data --field angular_velocity

# Difference should be the bias offset
```

### 6. Check Gyro Bias Estimation

Monitor bias estimation (runs continuously):

```bash
# Check gyro bias estimator node
ros2 node info /sensing/imu/gyro_bias_estimator

# Check estimated bias
ros2 topic echo /sensing/imu/gyro_bias_estimator/gyro_bias

# Output shows bias for each axis:
# x: 0.001  # rad/s
# y: -0.002
# z: 0.0005

# Check if bias is being applied
# Compare timestamps to ensure it's updating
ros2 topic hz /sensing/imu/gyro_bias_estimator/gyro_bias
```

**Bias Estimation Requirements**:
- Vehicle must be moving (uses odometry)
- Requires `/localization/kinematic_state` topic
- Takes time to converge (minutes)
- More accurate when vehicle drives straight

### 7. Check IMU Frame Transforms

Verify IMU coordinate frame:

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Look for imu_link in frames.pdf
# Expected chain: base_link → sensor_kit_base_link → imu_link

# Check specific transform
ros2 run tf2_ros tf2_echo base_link imu_link

# Should show rotation matching your calibration:
# Translation: [0, 0, 2.1]
# Rotation: [180°, 0°, 180°] (upside-down mount)

# Verify static transform is published
ros2 topic echo /tf_static | grep imu
```

**From your calibration** (`sensor_kit_calibration.yaml:16-22`):
```yaml
imu_link:
  x: 0.0
  y: 0.0
  z: 2.1
  roll: 3.14159   # 180° (upside down)
  pitch: 0.0
  yaw: 3.14159    # 180°
```

### 8. Check IMU Calibration Parameters

Verify IMU correction configuration:

```bash
# Find your IMU corrector config
# Default: individual_params/config/<vehicle_id>/nuway_sensor_kit/imu_corrector.param.yaml

# Check loaded parameters
ros2 param dump /sensing/imu/imu_corrector

# Key parameters:
# - angular_velocity_offset_x/y/z: Bias values (rad/s)
# - angular_velocity_stddev_xx/yy/zz: Noise standard deviation
```

**Default values** (from autoware_imu_corrector):
```yaml
angular_velocity_offset_x: 0.0    # Updated by bias estimator
angular_velocity_offset_y: 0.0
angular_velocity_offset_z: 0.0
angular_velocity_stddev_xx: 0.03  # Noise level
angular_velocity_stddev_yy: 0.03
angular_velocity_stddev_zz: 0.03
```

### 9. Test with Minimal Setup

Launch only IMU components:

```bash
# Source workspace
source /home/user/autoware.shuttle_bus/install/setup.bash

# Launch just IMU
ros2 launch nuway_sensor_kit_launch sensing.launch.xml \
  sensor_model:=nuway_sensor_kit \
  vehicle_id:=default \
  launch_driver:=true

# In another terminal, monitor IMU
ros2 topic echo /sensing/imu/imu_data
```

### 10. Check Localization Integration

Verify IMU is being used by localization:

```bash
# Check EKF localizer is running
ros2 node list | grep ekf_localizer

# Check if IMU data is subscribed
ros2 node info /localization/ekf_localizer | grep imu

# Should show subscription to /sensing/imu/imu_data

# Monitor localization output
ros2 topic echo /localization/kinematic_state --once

# Check twist (velocity) includes angular velocity from IMU
```

### 11. Verify IMU Driver Configuration

Check Tamagawa driver settings (if using):

**Launch file**: `nuway_sensor_kit_launch/launch/imu.launch.xml`

Key parameters:
- Device port: `/dev/imu`
- Frame ID: `tamagawa/imu_link`
- Topic: `imu/tamagawa/imu_raw`

**Note**: Driver section is commented out in default config, suggesting:
- IMU might be integrated with another sensor (GNSS+INS)
- Or using a different driver/interface
- Check your specific vehicle configuration

### 12. Check Diagnostics

Monitor IMU health:

```bash
# Check diagnostics
ros2 topic echo /diagnostics | grep -A 10 imu

# Look for:
# - IMU data rate
# - Bias estimation status
# - Error messages
# - Warning flags

# Check aggregated diagnostics
ros2 topic echo /diagnostics_agg | grep -A 10 imu
```

### 13. Manual IMU Calibration (if needed)

If bias estimation is not working, manually calibrate:

```bash
# 1. Place vehicle on level ground, engine off, stationary
# 2. Record raw IMU data for 60 seconds
ros2 topic echo /sensing/imu/tamagawa/imu_raw > imu_calibration.txt

# 3. Calculate average angular velocity (bias)
# Using Python or command-line tools

# 4. Update config file with bias values
# Edit: individual_params/config/<vehicle_id>/nuway_sensor_kit/imu_corrector.param.yaml
angular_velocity_offset_x: 0.0123  # Your calculated value
angular_velocity_offset_y: -0.0045
angular_velocity_offset_z: 0.0067

# 5. Restart Autoware to apply new values
```

## Common Issues and Solutions

### Issue 1: No IMU data (topics not publishing)

**Cause**: Driver not running or device not connected

**Solutions**:
```bash
# 1. Check device connection
ls -l /dev/imu
# If missing, check /dev/ttyUSB* or /dev/ttyACM*

# 2. Check driver is enabled
# Edit launch file, uncomment driver section if needed

# 3. Verify permissions
sudo chmod 666 /dev/imu

# 4. Check for errors in driver log
ros2 node info /sensing/imu/tamagawa/tag_serial_driver
```

### Issue 2: IMU data frozen (not updating)

**Cause**: Driver communication failure or sensor malfunction

**Solutions**:
```bash
# 1. Check message timestamps
ros2 topic echo /sensing/imu/tamagawa/imu_raw --field header.stamp

# If not incrementing, driver has stopped

# 2. Restart driver node
# Kill and relaunch Autoware

# 3. Check serial communication
# Verify baudrate, parity, stop bits match sensor
# Check for cable damage

# 4. Test with manufacturer's software
# Verify IMU works with native tools
```

### Issue 3: Incorrect orientation (Z acceleration not 9.8 m/s²)

**Cause**: Wrong mounting calibration or sensor orientation

**Solutions**:
```bash
# 1. Verify physical mounting
# Check IMU orientation matches calibration

# 2. Update sensor_kit_calibration.yaml
# Adjust roll, pitch, yaw for imu_link

# Example: If IMU is mounted upside down
# roll: 3.14159 (180°), pitch: 0, yaw: 3.14159

# 3. Check coordinate frame convention
# X: forward, Y: left, Z: up (ROS standard)

# 4. Restart to apply calibration changes
```

### Issue 4: Large bias drift (angular velocity not zero when stationary)

**Cause**: IMU bias not calibrated or temperature drift

**Solutions**:
```bash
# 1. Let IMU warm up (5-10 minutes)
# Bias changes with temperature

# 2. Enable gyro bias estimator
# Should run automatically if localization active

# 3. Manual calibration (see Section 13)
# Calculate and set bias offsets

# 4. Drive vehicle for bias estimation
# Estimator needs motion to converge
# Drive straight at constant speed for best results

# 5. Check IMU quality
# Low-cost IMUs have larger drift
# Consider upgrading to MEMS or FOG IMU
```

### Issue 5: Noisy/erratic IMU data

**Cause**: Vibration, electrical interference, or bad sensor

**Solutions**:
```bash
# 1. Check mechanical mounting
# IMU should be rigidly mounted
# Use vibration dampening if on engine

# 2. Check electrical shielding
# Keep away from motors, power cables
# Use shielded cables
# Proper grounding

# 3. Increase noise parameters
# Edit imu_corrector.param.yaml
# angular_velocity_stddev_xx: 0.05  # Increase if noisy

# 4. Add low-pass filtering (if needed)
# May require custom node or driver configuration
```

### Issue 6: IMU corrector not applying bias

**Cause**: Parameters not loaded or bias estimator not running

**Solutions**:
```bash
# 1. Check bias estimator is running
ros2 node list | grep gyro_bias

# 2. Verify parameter file exists
ls individual_params/config/*/nuway_sensor_kit/imu_corrector.param.yaml

# 3. Check parameters are loaded
ros2 param list /sensing/imu/imu_corrector

# 4. Manually set parameters
ros2 param set /sensing/imu/imu_corrector angular_velocity_offset_x 0.01

# 5. Check input topic to bias estimator
# Requires /localization/kinematic_state
ros2 topic list | grep kinematic_state
```

### Issue 7: Transform errors (imu_link not found)

**Cause**: Static transform not published or wrong frame name

**Solutions**:
```bash
# 1. Check TF tree
ros2 run tf2_tools view_frames

# 2. Verify frame ID in IMU messages
ros2 topic echo /sensing/imu/imu_data --field header.frame_id

# 3. Check sensor_kit_calibration.yaml
# Should define imu_link transform

# 4. Verify robot_state_publisher is running
ros2 node list | grep robot_state

# 5. Check URDF/xacro files include IMU
# Should have <link name="imu_link"/>
```

### Issue 8: "Permission denied" on /dev/imu

**Cause**: User lacks serial port permissions

**Solutions**:
```bash
# Temporary fix
sudo chmod 666 /dev/imu

# Permanent fix
sudo usermod -a -G dialout $USER
newgrp dialout  # Or logout/login

# Verify
groups  # Should include 'dialout'

# Create udev rule for persistent device name
# /etc/udev/rules.d/99-imu.rules:
# SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", SYMLINK+="imu"
sudo udevadm control --reload-rules
```

## Quick Diagnostic Command Summary

Run these commands in sequence:

```bash
# 1. Hardware
ls -l /dev/imu

# 2. ROS nodes
ros2 node list | grep imu

# 3. Topics
ros2 topic list | grep imu
ros2 topic hz /sensing/imu/imu_data

# 4. Data sanity check
ros2 topic echo /sensing/imu/imu_data --once

# 5. Bias estimation
ros2 topic echo /sensing/imu/gyro_bias_estimator/gyro_bias --once

# 6. Transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link imu_link

# 7. Diagnostics
ros2 topic echo /diagnostics | grep -A 5 imu
```

## Expected Working State

When IMU is working correctly:

1. ✅ Device `/dev/imu` exists and is accessible
2. ✅ `imu_corrector` and `gyro_bias_estimator` nodes running
3. ✅ `/sensing/imu/imu_data` publishing at 50-200 Hz
4. ✅ Angular velocity near 0 when stationary (within ±0.05 rad/s)
5. ✅ Linear acceleration Z near 9.8 m/s² when stationary
6. ✅ Bias estimation converging (check `/gyro_bias` topic)
7. ✅ Transform from `base_link` to `imu_link` exists
8. ✅ No errors in `/diagnostics`
9. ✅ EKF localizer subscribing to `/sensing/imu/imu_data`
10. ✅ Orientation changes when vehicle rotates

## IMU Data Interpretation

### Stationary Vehicle (Level Ground)
```
angular_velocity:
  x: 0.001  # ±0.05 rad/s acceptable
  y: -0.002
  z: 0.0001

linear_acceleration:
  x: 0.05   # Small, near 0
  y: -0.03  # Small, near 0
  z: 9.81   # Gravity (±0.5 acceptable)
```

### Moving Vehicle (Turning Left)
```
angular_velocity:
  x: 0.02   # Small (roll)
  y: -0.01  # Small (pitch)
  z: 0.35   # Yaw rate (turning)

linear_acceleration:
  x: 2.5    # Forward acceleration
  y: -1.8   # Centripetal (turning)
  z: 9.7    # Gravity component
```

## IMU Types and Characteristics

| IMU Type | Gyro Bias Drift | Noise | Update Rate | Cost | Use Case |
|----------|----------------|-------|-------------|------|----------|
| Consumer MEMS | 10-100 °/hr | High | 100 Hz | $ | Testing |
| Automotive MEMS | 1-10 °/hr | Medium | 100-200 Hz | $$ | Production vehicles |
| Tactical MEMS | 0.1-1 °/hr | Low | 200-400 Hz | $$$ | High-accuracy navigation |
| FOG (Fiber Optic) | <0.01 °/hr | Very Low | 100-1000 Hz | $$$$ | Aerospace, survey |

**Your Tamagawa IMU**: Likely automotive-grade MEMS (good balance of performance/cost)

## Calibration Best Practices

**Initial Calibration**:
1. Mount IMU rigidly to vehicle structure
2. Align axes with vehicle frame (or calibrate rotation)
3. Record bias on level ground, engine off, 5+ minutes
4. Calculate average angular velocity (bias)
5. Update `imu_corrector.param.yaml`

**Runtime Calibration**:
1. Enable gyro bias estimator (automatic)
2. Drive vehicle normally for 10-30 minutes
3. Bias converges to more accurate values
4. Most effective during straight driving

**Verification**:
- Check stationary angular velocity < 0.01 rad/s
- Monitor localization drift over time
- Compare against GNSS position

## Integration with Localization

**EKF Localizer** uses IMU for:
- **Prediction step**: Propagate state using angular velocity
- **Faster updates**: IMU runs at higher rate than GNSS
- **Smoothing**: Reduces position jitter

**GNSS+IMU Fusion**:
- IMU provides short-term accuracy (no drift propagation with correction)
- GNSS provides long-term accuracy (corrects IMU drift)
- Combined system is more robust than either alone

## Additional Resources

- **Tamagawa IMU**: Check manufacturer documentation for specifications
- **IMU Corrector Source**: `/home/user/autoware.shuttle_bus/src/universe/autoware_universe/sensing/autoware_imu_corrector/`
- **ROS 2 sensor_msgs/Imu**: http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
- **IMU Basics**: https://www.vectornav.com/resources/inertial-navigation-primer
- **Gyroscope Bias**: https://www.analog.com/en/analog-dialogue/articles/mems-imu-gyroscope-alignment.html

## Troubleshooting Checklist

- [ ] IMU device connected (`/dev/imu` exists)
- [ ] User has serial port permissions (`dialout` group)
- [ ] IMU securely mounted to vehicle
- [ ] `imu_corrector` node running
- [ ] `gyro_bias_estimator` node running
- [ ] `/sensing/imu/imu_data` publishing at 50+ Hz
- [ ] Angular velocity near 0 when stationary
- [ ] Linear acceleration Z near 9.8 m/s²
- [ ] Gyro bias estimation active
- [ ] Transform to `imu_link` exists
- [ ] Calibration matches physical mounting
- [ ] No errors in diagnostics
- [ ] EKF localizer using IMU data
- [ ] Localization drift acceptable

---

**Last Updated**: 2025-11-16
**Configuration**: nuway_sensor_kit with Tamagawa IMU
**Processing**: autoware_imu_corrector + gyro_bias_estimator
