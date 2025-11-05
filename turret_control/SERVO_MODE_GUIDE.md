# CubeMars Motor Control: MIT Mode vs Servo Modes

This guide explains the motor control modes available for the CubeMars AK60-6 motor.

## Overview

The CubeMars AK60-6 motor supports **8 total control modes**:
- **1 MIT Mode** (high-performance real-time control)
- **6 Servo Modes** (classical servo control with different strategies)

## MIT Mode (Original Implementation)

### Characteristics:
- **Real-time control**: Direct torque, position, and velocity control
- **High bandwidth**: Suitable for dynamic control applications
- **Standard CAN frames**: Uses motor ID as CAN identifier
- **Continuous feedback**: Real-time position, velocity, and torque feedback

### Usage:
```cpp
// Enter MIT mode
motor.enterMITMode();

// Send position command with gains
motor.sendCommandMITMode(
    1.57f,    // position (radians)
    0.0f,     // velocity (rad/s)  
    50.0f,    // Kp gain
    1.0f,     // Kd gain
    0.0f      // feedforward torque
);

// Exit MIT mode
motor.exitMITMode();
```

## Servo Modes (New Implementation)

### Characteristics:
- **Extended CAN frames**: Control mode encoded in CAN ID bits [28:8]
- **Mode-specific control**: Each mode optimized for specific use cases
- **Automatic feedback**: 8-byte periodic status messages
- **Parameter limits**: Built-in safety limits for each mode

### Mode Details:

#### 1. Duty Cycle Mode (0)
- **Purpose**: Direct voltage control via PWM duty cycle
- **Range**: -100% to +100% (-1.0 to 1.0)
- **Use case**: Low-level voltage control, testing

```cpp
motor.sendDutyCycleMode(0.5f);  // 50% duty cycle
```

#### 2. Current Loop Mode (1) 
- **Purpose**: Direct Iq current control (torque control)
- **Range**: -60A to +60A
- **Use case**: Torque control, force applications

```cpp
motor.sendCurrentLoopMode(5.0f);  // 5A current (torque)
```

#### 3. Current Brake Mode (2)
- **Purpose**: Position holding with braking current
- **Range**: 0A to 60A (positive only)
- **Use case**: Position locking, emergency stopping

```cpp
motor.sendCurrentBrakeMode(10.0f);  // 10A brake current
```

#### 4. Speed Loop Mode (3)
- **Purpose**: Speed control
- **Range**: -100,000 to +100,000 ERPM
- **Use case**: Constant speed applications

```cpp
motor.sendSpeedLoopMode(5000.0f);  // 5000 ERPM
```

#### 5. Position Loop Mode (4)
- **Purpose**: Position control at maximum speed
- **Range**: -36,000° to +36,000° (±100 turns)
- **Use case**: Point-to-point positioning

```cpp
motor.sendPositionLoopMode(90.0f);  // Move to 90 degrees
```

#### 6. Set Origin Mode (5)
- **Purpose**: Set zero reference point
- **Types**: 0=temporary, 1=permanent (dual encoder only)
- **Use case**: Calibration, homing

```cpp
motor.setOriginMode(0);  // Set temporary zero point
```

#### 7. Position-Speed Loop Mode (6)
- **Purpose**: Position control with speed and acceleration limits
- **Parameters**: Position (±36,000°), Speed (±32,768 ERPM), Acceleration (0-32,767)
- **Use case**: Smooth motion profiles

```cpp
motor.sendPositionSpeedLoopMode(
    180.0f,   // target position (degrees)
    5000,     // max speed (ERPM)
    30000     // acceleration
);
```

## Feedback Comparison

### MIT Mode Feedback:
```cpp
float pos = motor.getPosition();        // radians
float vel = motor.getVelocity();        // rad/s
float torque = motor.getTorque();       // Nm
float temp = motor.getTemperature();    // °C
int error = motor.getErrorFlag();
```

### Servo Mode Feedback:
```cpp
auto feedback = motor.getServoFeedback();
float pos = feedback.position_degrees;     // degrees
float speed = feedback.speed_erpm;         // ERPM
float current = feedback.current_amps;     // amperes
float temp = feedback.temperature_celsius; // °C
uint8_t error = feedback.error_code;
```

## When to Use Each Mode

### Use MIT Mode for:
- Real-time control applications
- Dynamic motion control
- Impedance control
- Force control with custom algorithms
- High-frequency control loops (>1kHz)

### Use Servo Modes for:
- **Duty Cycle**: Hardware testing, voltage characterization
- **Current Loop**: Simple torque control, load testing
- **Current Brake**: Position holding, safety stops
- **Speed Loop**: Constant speed applications (fans, pumps)
- **Position Loop**: Simple point-to-point moves
- **Position-Speed**: Smooth motion profiles, trajectory following

## Implementation Notes

1. **Mode Switching**: Cannot switch modes while motor is running - power cycle required
2. **CAN Protocol**: MIT uses standard frames, Servo uses extended frames
3. **Safety Limits**: All servo modes have built-in parameter clamping
4. **Feedback Rate**: Servo modes can be configured for 1-500Hz feedback
5. **Bus Assignment**: All modes use CAN bus 5 (JC5) at 1Mbps

## Error Codes (Servo Mode)

- `0`: No fault
- `1`: Motor over-temperature
- `2`: Over-current fault  
- `3`: Over-voltage fault
- `4`: Under-voltage fault
- `5`: Encoder fault
- `6`: MOSFET over-temperature
- `7`: Motor stall

## Example Usage

See `examples/servo_mode_demo.cpp` for a complete demonstration of all modes.