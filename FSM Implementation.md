
# G1 Robot FSM Implementation

## Overview

The Finite State Machine (FSM) has been implemented in the real-time thread to handle state transitions and generate appropriate motor commands based on the current FSM state.

## Files Added

1. **[src/g1_fsm.h](src/g1_fsm.h)** - FSM header defining context and API
2. **[src/g1_fsm.c](src/g1_fsm.c)** - FSM implementation with state logic
3. **[src/ec_rt_thread.c](src/ec_rt_thread.c)** - Modified to integrate FSM

## FSM States Implemented

The FSM implements all states defined in the unitree_sdk2 API:

### 1. FSM_ZERO_TORQUE (0)
- **Description**: Disables all motors, zero torque output
- **Use Case**: Safe mode, emergency stop
- **Motor Command**: KP=0, KD=0, enabled=false

### 2. FSM_DAMP (1)
- **Description**: Damping mode with low stiffness
- **Use Case**: Compliant mode for manual manipulation
- **Motor Command**: KP=5.0 (very low), KD=10.0 (high damping)

### 3. FSM_SQUAT (2)
- **Description**: Squatting posture
- **Joint Angles**:
  - Hip Pitch: -0.8 rad
  - Knee: 1.6 rad
  - Ankle: -0.8 rad
- **Transition**: 2 seconds smooth interpolation

### 4. FSM_SIT (3)
- **Description**: Sitting posture
- **Joint Angles**:
  - Hip Pitch: -1.57 rad (~90°)
  - Knee: 2.8 rad (~160°)
  - Ankle: -1.23 rad
- **Transition**: 2 seconds smooth interpolation

### 5. FSM_STAND_UP (4)
- **Description**: Transition to standing pose
- **Joint Angles**:
  - Hip Pitch: -0.3 rad
  - Knee: 0.6 rad
  - Ankle: -0.3 rad
- **Transition**: 2 seconds smooth interpolation

### 6. FSM_LOCOMOTION (500)
- **Description**: Active walking/movement mode
- **Features**:
  - Reads velocity commands (vx, vy, omega) from shared memory
  - Currently maintains standing pose
  - Logs velocity commands for debugging
  - **TODO**: Implement actual gait controller

## State Transition Logic

### Transition Flow

```
User Command (via DDS)
  → LocoServer updates shm_data->loco.fsm_id
  → RT Thread reads fsm_id in g1_fsm_update()
  → FSM checks for transition
  → Initiates smooth transition (2 seconds)
  → Updates motor commands during transition
  → Completes transition, enters stable state
```

### Smooth Transitions

All pose transitions use:
- **Cubic smoothstep interpolation** for natural movement
- **2-second duration** (configurable via TRANSITION_DURATION_NS)
- **Linear interpolation** between current and target poses
- **Progress tracking** (0.0 to 1.0)

## Motor Control Parameters

The FSM generates motor commands with appropriate gains:

- **Position Gain (KP)**: 100.0 for active control, 5.0 for damping
- **Velocity Gain (KD)**: 5.0 for active control, 10.0 for damping
- **Feedforward Torque**: 0.0 (not currently used)

## Integration with RT Thread

The FSM is integrated into the real-time cyclic task:

```c
// In rt_cyclic_task():
1. Receive EtherCAT data
2. Process motor states
3. Process hands
4. *** Update FSM and generate motor commands ***
5. Send EtherCAT commands
```

The FSM runs at the same frequency as the RT thread (500 Hz by default).

## Testing the FSM

### Prerequisites

```bash
# Terminal 1: Start RT thread
sudo ./build/bin/ec_rt_thread

# Terminal 2: Start DDS bridge
./build/bin/unitree_bridge
```

### Test Commands

```bash
# Zero torque (disable all motors)
./run_unitree_client.sh --zero_torque

# Damp mode (compliant)
./run_unitree_client.sh --damp

# Squat pose
./run_unitree_client.sh --squat

# Sit down
./run_unitree_client.sh --sit

# Stand up
./run_unitree_client.sh --stand_up

# Enter locomotion mode
./run_unitree_client.sh --start

# Verify current state
./run_unitree_client.sh --get_fsm_id
```

### Expected Console Output

From the RT thread terminal, you should see:

```
[FSM] Initialized in state: ZERO_TORQUE
[FSM] Transition: ZERO_TORQUE -> SQUAT
[FSM] Transition complete: SQUAT
[FSM] Transition: SQUAT -> STAND_UP
[FSM] Transition complete: STAND_UP
```

## Architecture

```
┌─────────────────────────────────────────┐
│         DDS Command (Client)            │
└───────────────┬─────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│      LocoServer (unitree_bridge)        │
│  - Receives API commands                │
│  - Updates shm_data->loco.fsm_id        │
└───────────────┬─────────────────────────┘
                │
                ▼ (Shared Memory)
┌─────────────────────────────────────────┐
│      RT Thread (ec_rt_thread)           │
│  ┌─────────────────────────────────┐    │
│  │  G1 FSM (g1_fsm.c)              │    │
│  │  - Reads fsm_id from shm        │    │
│  │  - Manages state transitions    │    │
│  │  - Generates motor commands     │    │
│  └─────────────────────────────────┘    │
│                │                        │
│                ▼                        │
│  ┌─────────────────────────────────┐    │
│  │  Motor Controllers              │    │
│  │  - SE motors (legs)             │    │
│  │  - LS motors (arms, waist)      │    │
│  │  - LS hands                     │    │
│  └─────────────────────────────────┘    │
└───────────────┬─────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│      EtherCAT Bus → Physical Robot      │
└─────────────────────────────────────────┘
```

## Implementation Notes

### Current Limitations

1. **Locomotion Mode**: Currently only maintains standing pose
   - **TODO**: Implement actual gait generator
   - **TODO**: Add CPG (Central Pattern Generator) for walking
   - **TODO**: Process velocity commands into leg trajectories

2. **Arm Control**: FSM only controls leg joints
   - **TODO**: Add arm pose definitions
   - **TODO**: Implement gesture controllers for shake_hand/wave_hand

3. **Safety**: No collision detection or joint limit checking
   - **TODO**: Add joint limit validation
   - **TODO**: Add self-collision avoidance

### Safety Features

- All transitions are smooth (2 seconds) to prevent sudden movements
- Zero torque mode can be triggered any time for emergency stop
- Motor commands are validated (joint ID bounds checking)

## Next Steps

1. **Gait Controller**: Implement walking gait for FSM_LOCOMOTION state
2. **Arm Gestures**: Add arm control for shake_hand and wave_hand tasks
3. **Parameter Tuning**: Adjust KP/KD gains based on actual robot performance
4. **State Validation**: Add checks for valid state transitions
5. **Feedback Control**: Use IMU data for balance control
6. **Force Control**: Integrate foot contact sensors for adaptive gait

## Configuration

Key parameters can be tuned in [src/g1_fsm.c](src/g1_fsm.c):

```c
#define TRANSITION_DURATION_NS (2000000000ULL)  // 2 seconds
#define TRANSITION_SMOOTHING 0.95f              // Smoothing factor

// Joint positions for poses
static const float POSE_SQUAT[6] = {...};
static const float POSE_SIT[6] = {...};
static const float POSE_STAND[6] = {...};
```

## Troubleshooting

### FSM doesn't transition
- Check RT thread is running: `ps aux | grep ec_rt_thread`
- Check shared memory exists: `ls -la /dev/shm/ec_motor_shm`
- Check DDS bridge is running and receiving commands
- Verify FSM logs in RT thread console

### Motors don't move
- Ensure motors are enabled in shared memory
- Check EtherCAT communication is working
- Verify motor gains (KP, KD) are not zero
- Check for EtherCAT errors in RT thread console

### Jerky movements
- Increase TRANSITION_DURATION_NS for slower transitions
- Adjust KD gain for more damping
- Check RT thread is running at correct frequency (500 Hz)

