# ROS2 Package and DDS Communication Fix

This document describes the fixes applied to resolve two issues with `unitree_ros2_example`.

## Issues Fixed

### Problem 1: `ros2 run` Package Not Found

**Symptom:**
```bash
$ ros2 run unitree_ros2_example g1_ls_hand_example L
Package 'unitree_ros2_example' not found
```

**Root Cause:** The `install(TARGETS)` commands in `CMakeLists.txt` were missing `DESTINATION lib/${PROJECT_NAME}`. ROS2 expects executables in `lib/<package_name>/`.

**Fix:** Updated all install commands in `CMakeLists.txt`:
```cmake
# Before
install(TARGETS g1_ls_hand_example)

# After  
install(TARGETS g1_ls_hand_example DESTINATION lib/${PROJECT_NAME})
```

---

### Problem 2: All Values Are 0

**Symptom:** Running the binary directly showed all sensor values as 0.

**Root Cause:** ROS2 rmw_cyclonedds automatically adds the `rt/` prefix to topic names. The working `g1_ankle_swing_example` uses `lowstate` which maps to `rt/lowstate` in raw DDS.

The hand example was using `rt/hand/left/state` in ROS2, which became `rt/rt/hand/left/state` in raw DDS (double prefix).

**Fix:** Updated `g1_ls_hand_example.cpp`:
1. Removed `rt/` prefix from topic names:
   - `rt/hand/left/state` → `hand/left/state`
   - `rt/hand/left/cmd` → `hand/left/cmd`
2. Switched to `SensorDataQoS()` matching the working ankle example

---

## Verification

```bash
$ source install/local_setup.bash
$ ros2 pkg executables unitree_ros2_example
unitree_ros2_example g1_ls_hand_example  # Now listed!

$ ros2 run unitree_ros2_example g1_ls_hand_example L
=== LEFT Hand State ===
Hand ID: 0  DOF: 6  OP: 0  Homed: 0

Motor States:
  Motor | Position | Velocity | Current | Status | Error
  ------|----------|----------|---------|--------|------
    0   |       1  |     500  |  30464  |   0x00 |  0x00
    ...

Sensor Data (Normal Force):
  [ 0]0.0 [ 1]257.2 ...
```

## Usage

1. Source the workspace:
   ```bash
   source /home/yctan/unitree_ros2/cyclonedds_ws/install/local_setup.bash
   source /home/yctan/unitree_ros2/example/install/local_setup.bash
   ```

2. Run the hand example:
   ```bash
   ros2 run unitree_ros2_example g1_ls_hand_example L  # Left hand
   ros2 run unitree_ros2_example g1_ls_hand_example R  # Right hand
   ```

3. Interactive commands:
   - `r` - Rotate (sinusoidal motion)
   - `g` - Grip (close fingers)
   - `p` - Print state
   - `s` - Stop motors
   - `q` - Quit

