# G1 Robot Redefinition - Implementation Walkthrough

## Summary
Redefined the Unitree G1 humanoid robot (29 DOF) with custom EtherCAT hardware using 4 masters controlling 29 motors. The implementation follows unitree_sdk2 joint ID conventions.

## Hardware Configuration

| Master | Motor Type | Count | Body Parts |
|--------|-----------|-------|------------|
| 0 | SE | 9 | Left Leg (6) + Waist (3) |
| 1 | SE | 6 | Right Leg (6) |
| 2 | LS | 7 | Left Arm (7) |
| 3 | LS | 7 | Right Arm (7) |

## Files Changed

### New Files
- `src/g1_robot_config.h` - G1 joint definitions, body parts, master mappings
- `src/unitree_hg.idl` - unitree_sdk2 compatible LowCmd/LowState messages
- `src/unitree_bridge.cpp` - Bridge between shared memory and unitree_sdk2

### Modified Files
- `src/constants.h` - Updated for 4 masters, 29 motors
- `src/motors.h` - Expanded master context with motor type
- `src/ec_shared_mem.h` - 29-motor array with joint IDs
- `src/ec_rt_thread.c` - 4-master RT thread with G1 joint mapping
- `src/ecrt_mock.c` - 4-master mock simulation
- `src/MotorControl.idl` - Added body parts, joint names
- `src/dds_bridge.cpp` - Updated for 29 motors with joint names

## Key Design Decisions

1. **Joint ID Convention**: Follows unitree_sdk2 `G1JointIndex` ordering (left leg → right leg → waist → left arm → right arm)

2. **Motor Types**: 
   - SE motors (simplified protocol) for legs and waist (masters 0-1)
   - LS motors (CiA 402 CST mode) for arms (masters 2-3)

3. **Shared Memory Layout**: Motors indexed by joint ID (0-28) for direct lookup

4. **Default Gains**: Uses unitree_sdk2 default Kp/Kd values per joint

## Build Instructions

### With Mock EtherCAT (simulation mode)
```bash
cd build
rm -rf * && cmake -DUSE_MOCK_ECAT=ON ..
make -j4
```

### With Real EtherCAT Hardware
```bash
cd build
rm -rf * && cmake ..
make -j4
```

**Build Targets:**
- `bin/ec_rt_thread` - Real-time EtherCAT control
- `bin/dds_bridge` - DDS communication bridge
- `bin/unitree_bridge` - unitree_sdk2 compatible bridge
- `lib/libec_shared_mem.so` - Shared memory library

## unitree_sdk2 Integration

### Message Structures (matching unitree_hg::msg::dds_)
| Structure | Description |
|-----------|-------------|
| MotorCmd_ | Per-motor command: mode, q, dq, tau, kp, kd |
| MotorState_ | Per-motor state: q, dq, ddq, tau_est, temperature |
| IMUState_ | IMU: quaternion, gyroscope, accelerometer, rpy |
| LowCmd_ | Full robot command (35 motors) |
| LowState_ | Full robot state (35 motors + IMU) |

### Topic Names
- `rt/lowcmd` - Subscribe for control commands
- `rt/lowstate` - Publish robot state

## Running the System

### Mock Mode (Testing)
```bash
# Terminal 1: Start RT thread
./bin/ec_rt_thread

# Terminal 2: Start unitree bridge
./bin/unitree_bridge

# Terminal 3: Run unitree_sdk2 example
./g1_ankle_swing_example enp3s0
```

### Real Hardware Mode
```bash
# Run with sudo for RT scheduling
sudo ./bin/ec_rt_thread
./bin/unitree_bridge
```

## Joint Mapping Reference

| Joint ID | Name | Master | Body Part |
|----------|------|--------|-----------|
| 0-5 | Left Leg | 0 | LeftHipPitch → LeftAnkleRoll |
| 6-11 | Right Leg | 1 | RightHipPitch → RightAnkleRoll |
| 12-14 | Waist | 0 | WaistYaw, WaistRoll, WaistPitch |
| 15-21 | Left Arm | 2 | LeftShoulderPitch → LeftWristYaw |
| 22-28 | Right Arm | 3 | RightShoulderPitch → RightWristYaw |

