# G1 Loco Server Implementation Summary

This document summarizes the implementation of missing command functions to match the `unitree_sdk2` `g1_loco_client_example.cpp`.

## Implementation Status

### ✅ Previously Implemented (Before)
- **Getters**: `GetFsmId`, `GetFsmMode`, `GetBalanceMode`, `GetSwingHeight`, `GetStandHeight`, `GetPhase`
- **Setters**: `SetFsmId`, `SetBalanceMode`, `SetSwingHeight`, `SetStandHeight`, `SetVelocity`, `SetTaskId`, `SetSpeedMode`

### ✅ Newly Implemented (Now)

#### State Transition Commands (FSM Control)
1. **Damp()** - API ID: 7201
   - Sets FSM to damping mode (FSM_DAMP = 1)
   - Use: Safe mode with reduced motor forces

2. **ZeroTorque()** - API ID: 7202
   - Sets FSM to zero torque mode (FSM_ZERO_TORQUE = 0)
   - Use: Completely disables motor torques

3. **Start()** - API ID: 7203
   - Sets FSM to locomotion mode (FSM_LOCOMOTION = 500)
   - Use: Enters active walking/movement mode

4. **Squat()** - API ID: 7204
   - Sets FSM to squatting pose (FSM_SQUAT = 2)
   - Use: Lower body posture

5. **Sit()** - API ID: 7205
   - Sets FSM to sitting pose (FSM_SIT = 3)
   - Use: Sitting down posture

6. **StandUp()** - API ID: 7206
   - Sets FSM to stand up transition (FSM_STAND_UP = 4)
   - Use: Transition from sitting/squatting to standing

#### Locomotion Commands
7. **StopMove()** - API ID: 7207
   - Sets all velocity commands to zero
   - Use: Emergency stop or halt movement

8. **HighStand()** - API ID: 7208
   - Sets stand_height to 0.7 (high stance)
   - Use: Elevated standing posture

9. **LowStand()** - API ID: 7209
   - Sets stand_height to 0.3 (low stance)
   - Use: Lowered standing posture

10. **BalanceStand()** - API ID: 7210
    - Enables balance_mode = 1
    - Use: Active balance control

11. **Move(vx, vy, omega)** - API ID: 7211
    - Sets immediate velocity command (continuous movement)
    - Accepts velocity as array or individual fields (vx, vy, omega)
    - Use: Direct velocity control

12. **ContinuousGait(bool)** - API ID: 7212
    - Toggles continuous gait mode via balance_mode
    - Accepts: true/false or 0/1
    - Use: Enable/disable continuous walking pattern

13. **SwitchMoveMode(bool)** - API ID: 7213
    - Toggles FSM mode between movement patterns
    - Accepts: true/false or 0/1
    - Use: Switch between different gait patterns

#### Gesture/Task Commands
14. **ShakeHand(int cmd)** - API ID: 7214
    - cmd=0: Start shake hand gesture (task_id=1)
    - cmd=1: Stop shake hand gesture (task_id=0)
    - Use: Handshaking interaction

15. **WaveHand(bool with_turn)** - API ID: 7215
    - with_turn=false: Simple wave (task_id=2)
    - with_turn=true: Wave with body turn (task_id=3)
    - Use: Waving gesture for interaction

## Files Modified

1. **[g1_service_types.h](src/g1_service_types.h:44)**
   - Added 15 new API ID definitions (7201-7215)

2. **[g1_loco_server.h](src/g1_loco_server.h:40)**
   - Added 15 new handler function declarations

3. **[g1_loco_server.cpp](src/g1_loco_server.cpp:47)**
   - Added 15 handler registrations in constructor
   - Implemented all 15 command handler functions

## Usage Example

```bash
# Example client calls (similar to g1_loco_client_example.cpp)

# State transitions
./g1_loco_client --damp              # Enter damping mode
./g1_loco_client --stand_up          # Stand up from sitting
./g1_loco_client --start             # Enter locomotion mode

# Standing postures
./g1_loco_client --high_stand        # High stance
./g1_loco_client --low_stand         # Low stance
./g1_loco_client --balance_stand     # Balance mode

# Movement
./g1_loco_client --move="0.5 0.0 0.1"           # Move forward with slight turn
./g1_loco_client --stop_move                     # Stop moving
./g1_loco_client --continous_gait=true           # Enable continuous gait

# Gestures
./g1_loco_client --shake_hand                    # Start handshake
./g1_loco_client --wave_hand                     # Wave hand
./g1_loco_client --wave_hand_with_turn           # Wave with turn
```

## Integration Notes

- All commands use the existing DDS service infrastructure via `unitree_bridge`
- Commands modify the `LocoState` structure in shared memory
- The real-time thread (ec_rt_thread) reads these values to execute the commands
- FSM state machine logic needs to be implemented in the RT thread to respond to FSM transitions

## Testing

Build and test the implementation:

```bash
# Build the bridge
cmake --build build --target unitree_bridge

# Run the bridge
sudo ./build/bin/unitree_bridge

# In another terminal, test with the client
./build/bin/g1_loco_client_test
```

## Next Steps

1. Implement FSM state machine logic in RT thread to handle state transitions
2. Add actual gait controller to respond to movement commands
3. Implement arm gesture controllers for shake_hand and wave_hand
4. Add phase computation in GetPhase handler based on gait state

