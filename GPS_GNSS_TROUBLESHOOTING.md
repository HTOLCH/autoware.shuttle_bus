# GPS/GNSS Troubleshooting Guide

## Issue: GPS/GNSS not providing position data or poor localization

This guide will help you systematically debug GPS/GNSS issues on your shuttle bus running Autoware.

## Your Configuration (from analysis)

Based on your `nuway_sensor_kit_launch` configuration:

- **GNSS Receiver Options**:
  - **u-blox** (default) - C94-M8P or F9P module
  - **Septentrio** (alternative) - Mosaic receiver

- **Default Setup**:
  - Driver: `ublox_gps` package
  - Config: `c94_f9p_rover.yaml`
  - Raw topic: `/gnss/ublox/nav_sat_fix`
  - Orientation: `/autoware_orientation` (GNSS+INS)
  - Converter: `autoware_gnss_poser` (NavSatFix → MGRS Pose)

- **Output Topics**:
  - `/gnss/pose` - GNSS position
  - `/gnss/pose_with_covariance` - Position with uncertainty
  - `/gnss/fixed` - RTK fix status

## GNSS Basics

**Fix Types**:
- **No Fix (0)**: No satellites, no position
- **GPS Fix (1)**: Standard GPS, ~2-5m accuracy
- **DGPS (2)**: Differential GPS, ~1m accuracy
- **RTK Float (5)**: RTK initializing, ~0.5m accuracy
- **RTK Fixed (4)**: RTK converged, ~2cm accuracy

**Required for Autoware**:
- Minimum: GPS Fix (for rough position)
- Recommended: RTK Fixed (for precise localization)

## Step-by-Step Debugging

### 1. Hardware Connection Check

Verify physical connections:

```bash
# Check if GNSS is connected via USB/Serial
ls /dev/tty* | grep -E "(USB|ACM)"

# Common devices:
# /dev/ttyUSB0 - USB-Serial adapter
# /dev/ttyACM0 - Direct USB connection

# Check permissions
ls -l /dev/ttyUSB0  # or your device

# Fix permissions if needed
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER
# Then logout/login
```

**Expected**: You should see a device like `/dev/ttyUSB0` or `/dev/ttyACM0`

### 2. Antenna and Signal Check

**Physical checks**:
- Antenna is mounted on vehicle roof (clear sky view)
- Antenna cable is securely connected
- No metal obstructions above antenna
- Antenna has power (active antennas need external power)

**Sky view requirements**:
- Minimum 4 satellites for 3D fix
- Minimum 5 satellites for RTK
- Open sky view (no buildings, trees, tunnels)

### 3. Check ROS 2 GNSS Nodes

After launching Autoware:

```bash
# List all nodes - look for GNSS nodes
ros2 node list | grep gnss

# Expected output:
# /sensing/gnss/ublox  (or septentrio)
# /sensing/gnss/gnss_poser

# Check node info
ros2 node info /sensing/gnss/ublox
```

**If nodes are missing**:
- Check `launch_driver` is set to `true` in launch command
- Verify device permissions (Step 1)
- Check Autoware launch logs for errors

### 4. Verify GNSS Topics

Check if GNSS data is being published:

```bash
# List topics
ros2 topic list | grep gnss

# Expected topics:
# /sensing/gnss/ublox/nav_sat_fix (raw GNSS)
# /sensing/gnss/pose (converted pose)
# /sensing/gnss/pose_with_covariance
# /sensing/gnss/fixed (RTK status)

# Check raw GNSS data
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix --once

# Expected fields:
# - latitude, longitude, altitude
# - status.status: 0=no fix, 1=fix, 2=SBAS, 4=RTK fixed
# - position_covariance: uncertainty values

# Check message rate (should be 1-10 Hz)
ros2 topic hz /sensing/gnss/ublox/nav_sat_fix

# Check GNSS pose output
ros2 topic echo /sensing/gnss/pose --once
```

### 5. Check GNSS Fix Status

Monitor fix quality:

```bash
# Echo raw NavSatFix message
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix

# Look for:
# status:
#   status: 0  # -1=no fix, 0=fix, 1=SBAS, 2=GBAS/RTK
#   service: 1 # GPS service available
#
# latitude: 35.xxx  # Your location
# longitude: 139.xxx
# altitude: 50.0
#
# position_covariance: [x, 0, 0, 0, y, 0, 0, 0, z]  # Lower is better

# Check RTK fix status (if using RTK)
ros2 topic echo /sensing/gnss/fixed

# Should be True when RTK fixed
```

**Fix quality indicators**:
- `status.status == -1`: No fix (no satellites)
- `status.status == 0`: Standard GPS fix (OK for rough position)
- `status.status == 2`: RTK fix (best for precise localization)
- Small `position_covariance` values: Good accuracy (~0.01 for RTK, ~25 for GPS)

### 6. Check Satellite Count and Signal Strength

For u-blox receivers:

```bash
# Monitor satellite data (if available)
ros2 topic echo /sensing/gnss/ublox/navsat

# Or check via u-center software (Windows tool from u-blox)
# Connect to /dev/ttyUSB0 with u-center to see:
# - Number of satellites
# - Signal strength (C/N0)
# - Fix type
# - HDOP/VDOP values
```

**Good values**:
- Satellites: 8+ visible, 5+ used
- C/N0: >35 dBHz
- HDOP: <2.0 (horizontal dilution of precision)

### 7. Verify GNSS Poser Conversion

Check if NavSatFix is being converted to pose correctly:

```bash
# Check gnss_poser node
ros2 node info /sensing/gnss/gnss_poser

# Verify coordinate conversion is working
ros2 topic echo /sensing/gnss/pose --once

# Should show MGRS grid coordinates or local frame coordinates
# position:
#   x: 123.45
#   y: 678.90
#   z: 50.0
```

### 8. Check GNSS Orientation (for GNSS+INS)

If using dual GNSS or GNSS+INS for heading:

```bash
# Check orientation topic
ros2 topic echo /autoware_orientation --once

# Should contain quaternion orientation
# orientation:
#   x: 0.0
#   y: 0.0
#   z: 0.707
#   w: 0.707

# Check if orientation is being used
ros2 param get /sensing/gnss/gnss_poser use_gnss_ins_orientation
# Should return: true
```

### 9. Check Transform (TF) for GNSS

Verify GNSS frame transform:

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Look for gnss_link or gnss frame in frames.pdf
# Should have: base_link → sensor_kit_base_link → gnss_link

# Check specific transform
ros2 run tf2_ros tf2_echo base_link gnss_link

# Verify transform is being published
ros2 topic echo /tf_static | grep gnss
```

**From your calibration** (`sensor_kit_calibration.yaml`):
- GNSS frame should be defined relative to `sensor_kit_base_link`
- Check if `gnss_link` or similar frame exists

### 10. Test with Minimal Setup

Launch only GNSS components:

```bash
# Source workspace
source /home/user/autoware.shuttle_bus/install/setup.bash

# Launch just GNSS
ros2 launch nuway_sensor_kit_launch sensing.launch.xml \
  sensor_model:=nuway_sensor_kit \
  vehicle_id:=default \
  launch_driver:=true

# In another terminal, monitor GNSS
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix
```

### 11. Check RTK Correction Data (if using RTK)

For RTK operation, you need correction data from a base station:

```bash
# Check if receiving RTCM corrections
# (u-blox can receive via serial, TCP, or NTRIP)

# If using NTRIP client, check connection
ros2 topic echo /sensing/gnss/rtcm  # If available

# Check RTK age of differential
# Should be <5 seconds for good RTK
```

**RTK Setup Requirements**:
- Base station within 10-20km
- Correction data stream (RTCM3 messages)
- Good base station position (surveyed)
- Clear line-of-sight to satellites

### 12. Verify u-blox Configuration

Check receiver configuration (via u-center or UBX protocol):

**Key settings**:
- Baud rate: 115200 (or as configured)
- Update rate: 1-10 Hz
- GNSS systems enabled: GPS + GLONASS + Galileo + BeiDou
- Messages enabled:
  - UBX-NAV-PVT (position/velocity/time)
  - UBX-NAV-SAT (satellite info)
  - NMEA-GGA, RMC (if needed)

**For RTK (F9P)**:
- Operation mode: Rover
- Correction input: RTCM3 (via UART2, USB, or TCP)
- Survey-in disabled (for rover)

### 13. Review Launch Configuration

Verify parameters in launch file:

**File**: `/home/user/autoware.shuttle_bus/src/launcher/autoware_launch/sensor_kit/nuway_sensor_kit_launch/nuway_sensor_kit_launch/launch/gnss.launch.xml`

Key parameters:
- `gnss_receiver`: "ublox" or "septentrio"
- `launch_driver`: true
- Config file: `c94_f9p_rover.yaml` (for u-blox F9P)

**u-blox config file** (find in ublox_gps package):
```bash
# Find config
find /home/user/autoware.shuttle_bus/src -name "c94_f9p_rover.yaml"

# Check contents - should specify:
# - device: /dev/ttyUSB0 (or your device)
# - baudrate: 115200
# - frame_id: gnss_link
# - rate: 5 (Hz)
```

### 14. Check Diagnostics

```bash
# Monitor GNSS diagnostics
ros2 topic echo /diagnostics | grep -A 10 gnss

# Look for:
# - Fix status
# - Number of satellites
# - HDOP/VDOP
# - Error messages

# Check aggregated diagnostics
ros2 topic echo /diagnostics_agg | grep -A 10 gnss
```

## Common Issues and Solutions

### Issue 1: "No fix" or status=-1

**Cause**: No satellite signals received

**Solutions**:
```bash
# 1. Check antenna connection
# - Verify cable is connected
# - Check for damage to cable
# - Ensure active antenna has power

# 2. Check receiver is getting power
# - LED indicators on receiver (if any)
# - Check USB connection

# 3. Move to open area
# - Away from buildings
# - Clear sky view
# - Wait 2-5 minutes for initial fix (cold start)

# 4. Check receiver configuration
# - Ensure GPS system is enabled
# - Check antenna configuration (active vs passive)
```

### Issue 2: GPS fix but poor accuracy (large covariance)

**Cause**: Weak signals or poor satellite geometry

**Solutions**:
- Wait for more satellites (need 8+ for good accuracy)
- Move away from buildings/trees (multipath interference)
- Check for interference sources (RF noise)
- Consider upgrading to RTK for better accuracy

### Issue 3: No RTK fix (stuck at GPS or RTK Float)

**Cause**: Missing or poor correction data

**Solutions**:
```bash
# 1. Verify correction data source
# - Check NTRIP client connection
# - Verify base station is streaming
# - Check network connectivity

# 2. Check base station distance
# - Should be <20km for best results
# - Longer baseline = longer convergence time

# 3. Wait for convergence
# - RTK float → fixed can take 1-20 minutes
# - Requires stable satellite tracking
# - Movement helps convergence

# 4. Check correction data age
# - Should be <5 seconds
# - Older data = degraded accuracy
```

### Issue 4: "/dev/ttyUSB0: Permission denied"

**Cause**: User doesn't have serial port permissions

**Solutions**:
```bash
# Temporary fix (resets on reboot)
sudo chmod 666 /dev/ttyUSB0

# Permanent fix
sudo usermod -a -G dialout $USER
newgrp dialout  # Or logout/login

# Verify
groups  # Should include 'dialout'
```

### Issue 5: "Could not open port /dev/ttyUSB0"

**Cause**: Wrong device or device not found

**Solutions**:
```bash
# List all serial devices
ls /dev/tty* | grep -E "(USB|ACM)"

# Check which device is GNSS
dmesg | grep -i usb | tail -20

# Update launch config with correct device
# Edit ublox config: device: "/dev/ttyACM0"  # or your device
```

### Issue 6: GNSS pose not appearing in localization

**Cause**: GNSS poser not converting or bad coordinates

**Solutions**:
```bash
# Check gnss_poser is running
ros2 node list | grep gnss_poser

# Verify input/output topics
ros2 topic list | grep gnss

# Check for error messages
ros2 node info /sensing/gnss/gnss_poser

# Verify coordinate frame is correct
# - Should output in map frame or local coordinate system
# - Check map origin is set correctly
```

### Issue 7: Position jumps or jitters

**Cause**: Multipath, poor satellite geometry, or interference

**Solutions**:
- Improve antenna placement (higher, less obstruction)
- Use RTK for better accuracy
- Enable GNSS+INS fusion (smoother output)
- Check for RF interference sources
- Verify antenna is securely mounted (not vibrating)

### Issue 8: Septentrio receiver not working

**Cause**: Wrong driver or configuration

**Solutions**:
```bash
# Switch to Septentrio driver
# In launch command or config, set:
# gnss_receiver:=septentrio

# Verify septentrio driver is installed
ros2 pkg list | grep septentrio

# Install if missing
rosdep install -y --from-paths src --ignore-src

# Check Septentrio-specific topics
ros2 topic list | grep septentrio
```

## Quick Diagnostic Command Summary

Run these commands in sequence:

```bash
# 1. Hardware
ls /dev/tty* | grep -E "(USB|ACM)"

# 2. ROS nodes
ros2 node list | grep gnss

# 3. Topics
ros2 topic list | grep gnss
ros2 topic hz /sensing/gnss/ublox/nav_sat_fix

# 4. Fix status
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix --once

# 5. Pose output
ros2 topic echo /sensing/gnss/pose --once

# 6. Diagnostics
ros2 topic echo /diagnostics | grep -A 5 gnss

# 7. Transforms
ros2 run tf2_tools view_frames
```

## Expected Working State

When GNSS is working correctly:

1. ✅ Device appears as `/dev/ttyUSB0` or `/dev/ttyACM0`
2. ✅ `/sensing/gnss/ublox` node is running
3. ✅ `/sensing/gnss/ublox/nav_sat_fix` publishing at 1-10 Hz
4. ✅ `status.status >= 0` (has fix)
5. ✅ Latitude/longitude values are correct for your location
6. ✅ Position covariance is small (<25 for GPS, <1 for RTK)
7. ✅ `/sensing/gnss/pose` contains valid position
8. ✅ 8+ satellites visible
9. ✅ (If RTK) `status.status == 2` and `/gnss/fixed == True`

## GNSS Performance Guide

| Fix Type | Accuracy | Covariance | Requirements | Use Case |
|----------|----------|------------|--------------|----------|
| No Fix | N/A | N/A | 0 sats | Indoor/blocked |
| GPS Fix | 2-5m | ~25 | 4+ sats | Rough position |
| DGPS | ~1m | ~5 | 5+ sats + SBAS | Improved accuracy |
| RTK Float | 0.3-1m | ~2 | 5+ sats + corrections | Initializing |
| RTK Fixed | 2-5cm | ~0.01 | 5+ sats + good corrections | Precise localization |

## RTK Configuration Notes

For RTK operation with u-blox F9P:

**Correction Sources**:
1. **NTRIP Caster** (Internet-based):
   - Requires NTRIP client software
   - Publishes RTCM to receiver via serial/USB
   - Example: RTK2GO, UNAVCO, local services

2. **Radio Link** (Direct):
   - Base station with radio transmitter
   - Rover with radio receiver
   - No internet needed

3. **Local Base Station**:
   - Your own base station
   - Survey-in mode for accuracy
   - RTCM3 output to rover

**Configuration**:
```yaml
# Example RTCM input configuration for u-blox
# Via serial port (UART2):
# - Baudrate: 115200
# - Protocol: RTCM3
# - Messages: 1005, 1077, 1087, 1097, 1127, 1230
```

## Additional Resources

- **u-blox Documentation**: https://www.u-blox.com/en/product/zed-f9p-module
- **u-center Software**: Download from u-blox website (Windows GUI for configuration)
- **ublox_gps ROS Package**: https://github.com/KumarRobotics/ublox
- **Septentrio Documentation**: https://www.septentrio.com/
- **Autoware GNSS Poser**: `/home/user/autoware.shuttle_bus/src/core/autoware_core/sensing/autoware_gnss_poser/`
- **RTK Setup Guide**: https://learn.sparkfun.com/tutorials/what-is-gps-rtk

## Troubleshooting Checklist

- [ ] GNSS device connected and visible (`/dev/ttyUSB0`)
- [ ] User has serial port permissions (`dialout` group)
- [ ] Antenna mounted on roof with clear sky view
- [ ] Antenna cable securely connected
- [ ] `ublox_gps` node running
- [ ] `/sensing/gnss/ublox/nav_sat_fix` publishing
- [ ] Fix status >= 0 (has GPS fix)
- [ ] 8+ satellites visible
- [ ] Position covariance reasonable (<25)
- [ ] Latitude/longitude correct for location
- [ ] `/sensing/gnss/pose` contains valid data
- [ ] (RTK) Corrections being received
- [ ] (RTK) Fix status == 2 (RTK fixed)
- [ ] No error messages in diagnostics
- [ ] Transform to GNSS frame exists

---

**Last Updated**: 2025-11-16
**Configuration**: nuway_sensor_kit with u-blox (default) or Septentrio GNSS
**Driver**: ublox_gps or septentrio_gnss_driver
