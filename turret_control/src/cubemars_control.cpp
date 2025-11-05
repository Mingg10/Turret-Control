#include "cubemars_control.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>

// Constructor: allocate persistent buffers.
CubemarsControl::CubemarsControl(int motor_id, int can_bus, mjbots::pi3hat::Pi3Hat* pi3hat)
    : motor_id_(motor_id), can_bus_(can_bus), pi3hat_(pi3hat)
{
    // We use a persistent buffer with 5 slots (as in your earlier design).
    tx_can_.resize(5);
    rx_can_.resize(5);
}

// Helper function: Convert float x to fixed-point unsigned integer.
int CubemarsControl::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    x = std::clamp(x, x_min, x_max);
    // Replicate the conversion formula from your old code.
    return static_cast<int>((x - x_min) * ((1 << bits) / span));
}

// Helper function: Convert fixed-point unsigned int back to float.
float CubemarsControl::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return static_cast<float>(x_int) * span / ((1 << bits) - 1) + x_min;
}

// Synchronous command sending: build the TX frame, then call Cycle().
void CubemarsControl::sendCommandMITMode(float pos, float vel, float kp, float kd, float torq) {
    // Clamp inputs to the ranges from the manual.
    float p_des  = std::clamp(pos,  P_MIN, P_MAX);
    float v_des  = std::clamp(vel,  V_MIN, V_MAX);
    float kp_des = std::clamp(kp,   KP_MIN, KP_MAX);
    float kd_des = std::clamp(kd,   KD_MIN, KD_MAX);
    float t_ff   = std::clamp(torq, T_MIN, T_MAX);

    int con_pos  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int con_vel  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int con_kp   = float_to_uint(kp_des, KP_MIN, KP_MAX, 12);
    int con_kd   = float_to_uint(kd_des, KD_MIN, KD_MAX, 12);
    int con_torq = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    // Update TX buffer in slot can_bus_.
    // For JC5 (low-speed), we force frame.bus = 5.
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;  // Standard identifier (do not use extended flag)
    frame.bus = 5;         // Force JC5 (1Mbps classical CAN)
    frame.size = 8;
    frame.expect_reply = true;
    // Pack data as described in the manual (same as older code):
    frame.data[0] = static_cast<uint8_t>(con_pos >> 8);
    frame.data[1] = static_cast<uint8_t>(con_pos & 0xFF);
    frame.data[2] = static_cast<uint8_t>(con_vel >> 4);
    frame.data[3] = static_cast<uint8_t>(((con_vel & 0xF) << 4) | (con_kp >> 8));
    frame.data[4] = static_cast<uint8_t>(con_kp & 0xFF);
    frame.data[5] = static_cast<uint8_t>(con_kd >> 4);
    frame.data[6] = static_cast<uint8_t>(((con_kd & 0xF) << 4) | (con_torq >> 8));
    frame.data[7] = static_cast<uint8_t>(con_torq & 0xFF);

    // Build the input structure using our persistent buffers.
    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);  // JC5 is forced.
    input.timeout_ns = 1200000;
    
    // Call Cycle() synchronously.
    auto output = pi3hat_->Cycle(input);
    if (output.timeout) {
        std::cout << "Command timeout" << std::endl;
    }
    // Process the reply, if any.
    for (size_t i = 0; i < output.rx_can_size; i++) {
        if ( (rx_can_[i].id & 0x7FF) == static_cast<uint32_t>(motor_id_) ||
             rx_can_[i].data[0] == static_cast<uint8_t>(motor_id_) ) {
            unpackReply(rx_can_[i]);
        }
    }
}

void CubemarsControl::enterMITMode() {
    // Build the special packet: {0xFF,...,0xFF, 0xFC}
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFC;
    frame.expect_reply = false;
    
    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;
    
    auto output = pi3hat_->Cycle(input);
    if (output.timeout) {
        std::cout << "Enter MIT mode timeout" << std::endl;
    }
}

void CubemarsControl::exitMITMode() {
    // Build the special packet: {0xFF,..., 0xFD}
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFD;
    frame.expect_reply = false;
    
    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;
    
    auto output = pi3hat_->Cycle(input);
    if (output.timeout) {
        std::cout << "Exit MIT mode timeout" << std::endl;
    }
}


void CubemarsControl::zeroMotor() {
    // Build the special packet: {0xFF,...,0xFE}
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    frame.id = motor_id_;
    frame.bus = 5;
    frame.size = 8;
    for (int i = 0; i < 7; i++) {
        frame.data[i] = 0xFF;
    }
    frame.data[7] = 0xFE;
    frame.expect_reply = false;
    
    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);
    input.timeout_ns = 1200000;
    
    auto output = pi3hat_->Cycle(input);
    if (output.timeout) {
        std::cout << "Zero motor timeout" << std::endl;
    }
}

// Unpack a reply (following the manual's provided example).
bool CubemarsControl::unpackReply(const mjbots::pi3hat::CanFrame &frame) {
    // Expect an 8-byte frame.
    if (frame.size != 8)
        return false;
    
    // Check that the first data byte matches the motor id.
    if (frame.data[0] != static_cast<uint8_t>(motor_id_)) {
        std::cout << "Motor ID mismatch. Got: " << static_cast<int>(frame.data[0])
                  << ", Expected: " << motor_id_ << std::endl;
        return false;
    }
    
    // Extract fields exactly as in the older implementation.
    // Position: bytes [1]-[2] (16-bit big-endian)
    int p_int = (frame.data[1] << 8) | frame.data[2];
    // Speed: 12-bit value from byte 3 (high 8 bits) and upper nibble of byte 4.
    int v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);
    // Current (as torque): 12-bit value from lower nibble of byte 4 and all of byte 5.
    int i_int = ((frame.data[4] & 0x0F) << 8) | frame.data[5];
    
    // Conversion functions are expected to match the older ones.
    motor_data_.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    // Note: In the older code the field was named "speed" instead of "velocity".
    motor_data_.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_data_.torque   = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    
    // Temperature is given in byte 6 with an offset of 40.
    motor_data_.temperature = static_cast<float>(frame.data[6]) - 40;
    motor_data_.error_flag = frame.data[7];
    
    return true;
}


float CubemarsControl::getPosition() const { return motor_data_.position; }
float CubemarsControl::getVelocity() const { return motor_data_.velocity; }
float CubemarsControl::getTorque() const { return motor_data_.torque; }
float CubemarsControl::getTemperature() const { return motor_data_.temperature; }
int CubemarsControl::getErrorFlag() const { return motor_data_.error_flag; }

// ============================================================================
// SERVO MODE IMPLEMENTATION
// ============================================================================

// Helper functions for data packing (from CubeMars manual)
void CubemarsControl::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void CubemarsControl::buffer_append_int16(uint8_t* buffer, int16_t number, int32_t* index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// Generic servo command sender using extended CAN frame format
void CubemarsControl::sendServoCommand(ServoMode mode, const uint8_t* data, size_t data_len) {
    mjbots::pi3hat::CanFrame &frame = tx_can_[can_bus_];
    
    // Servo mode uses extended CAN frame format: 
    // CAN ID bits [28:8] = Control mode, [7:0] = Source node ID
    // Ensure extended frame by using bits beyond 11-bit standard range
    uint32_t control_mode = static_cast<uint32_t>(mode);
    frame.id = (control_mode << 8) | static_cast<uint32_t>(motor_id_) | 0x18000000;
    frame.bus = 5;  // Force JC5 (1Mbps classical CAN)
    frame.size = data_len;
    frame.expect_reply = true;

    // Copy data payload
    std::memcpy(frame.data, data, std::min(data_len, static_cast<size_t>(8)));
    
    // Send command and process reply
    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
    input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
    input.force_can_check = (1 << 5);  // JC5
    input.timeout_ns = 1200000;
    
    auto output = pi3hat_->Cycle(input);
    if (output.timeout) {
        std::cout << "Servo command timeout" << std::endl;
    }
    
    // Process any replies
    for (size_t i = 0; i < output.rx_can_size; i++) {
        if ((rx_can_[i].id & 0x7FF) == static_cast<uint32_t>(motor_id_) ||
            rx_can_[i].data[0] == static_cast<uint8_t>(motor_id_)) {
            unpackServoReply(rx_can_[i]);
        }
    }
}

// 1. Duty Cycle Mode (0) - Square wave voltage control
void CubemarsControl::sendDutyCycleMode(float duty_cycle) {
    duty_cycle = std::clamp(duty_cycle, -1.0f, 1.0f);  // -100% to +100%
    
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(duty_cycle * 100000.0f), &send_index);
    
    sendServoCommand(ServoMode::DUTY_CYCLE, buffer, 4);
}

// 2. Current Loop Mode (1) - Iq current control (torque)
void CubemarsControl::sendCurrentLoopMode(float current_amps) {
    current_amps = std::clamp(current_amps, -60.0f, 60.0f);  // Per manual: -60A to +60A
    
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(current_amps * 1000.0f), &send_index);
    
    sendServoCommand(ServoMode::CURRENT_LOOP, buffer, 4);
}

// 3. Current Brake Mode (2) - Braking current to fix position
void CubemarsControl::sendCurrentBrakeMode(float brake_current_amps) {
    brake_current_amps = std::clamp(brake_current_amps, 0.0f, 60.0f);  // 0A to 60A
    
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(brake_current_amps * 1000.0f), &send_index);
    
    sendServoCommand(ServoMode::CURRENT_BRAKE, buffer, 4);
}

// 4. Speed Loop Mode (3) - Speed control
void CubemarsControl::sendSpeedLoopMode(float speed_erpm) {
    speed_erpm = std::clamp(speed_erpm, -100000.0f, 100000.0f);  // Per manual: -100000 to 100000 ERPM
    
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(speed_erpm), &send_index);
    
    sendServoCommand(ServoMode::SPEED_LOOP, buffer, 4);
}

// 5. Position Loop Mode (4) - Position control at max speed  
void CubemarsControl::sendPositionLoopMode(float position_degrees) {
    position_degrees = std::clamp(position_degrees, -36000.0f, 36000.0f);  // Per manual: -36000° to 36000°
    
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(position_degrees * 10000.0f), &send_index);
    
    sendServoCommand(ServoMode::POSITION_LOOP, buffer, 4);
}

// 6. Set Origin Mode (5) - Set zero reference point
void CubemarsControl::setOriginMode(uint8_t origin_type) {
    uint8_t buffer[1];
    buffer[0] = origin_type;  // 0=temporary origin, 1=permanent zero point
    
    sendServoCommand(ServoMode::SET_ORIGIN, buffer, 1);
}

// 7. Position-Speed Loop Mode (6) - Position with speed and acceleration limits
void CubemarsControl::sendPositionSpeedLoopMode(float position_degrees, int16_t speed_erpm, int16_t acceleration) {
    position_degrees = std::clamp(position_degrees, -36000.0f, 36000.0f);
    speed_erpm = std::clamp(speed_erpm, static_cast<int16_t>(-32768), static_cast<int16_t>(32767));  // 16-bit signed range
    acceleration = std::clamp(acceleration, static_cast<int16_t>(0), static_cast<int16_t>(32767));    // 16-bit positive range
    
    uint8_t buffer[8];
    int32_t send_index = 0;
    
    // Position (32-bit)
    buffer_append_int32(buffer, static_cast<int32_t>(position_degrees * 10000.0f), &send_index);
    
    // Speed (16-bit) 
    buffer_append_int16(buffer, static_cast<int16_t>(speed_erpm / 10), &send_index);
    
    // Acceleration (16-bit)
    buffer_append_int16(buffer, static_cast<int16_t>(acceleration / 10), &send_index);
    
    sendServoCommand(ServoMode::POSITION_SPEED_LOOP, buffer, 8);
}

// Unpack servo mode reply (8-byte periodic upload format from manual)
bool CubemarsControl::unpackServoReply(const mjbots::pi3hat::CanFrame &frame) {
    if (frame.size != 8) {
        return false;
    }
    
    // Extract servo feedback data (format from manual page 32)
    // Position: int16 range -32000~32000 represents -3200°~3200°
    int16_t pos_int = (frame.data[0] << 8) | frame.data[1];
    servo_feedback_.position_degrees = static_cast<float>(pos_int) * 0.1f;
    
    // Speed: int16 range -32000~32000 represents -320000~320000 ERPM
    int16_t spd_int = (frame.data[2] << 8) | frame.data[3];  
    servo_feedback_.speed_erpm = static_cast<float>(spd_int) * 10.0f;
    
    // Current: int16 range -6000~6000 represents -60~60A
    int16_t cur_int = (frame.data[4] << 8) | frame.data[5];
    servo_feedback_.current_amps = static_cast<float>(cur_int) * 0.01f;
    
    // Temperature: int8 range -20~127°C with driver board temperature
    servo_feedback_.temperature_celsius = static_cast<float>(frame.data[6]);
    
    // Error code: uint8 (0=no fault, 1=over-temp, 2=over-current, etc.)
    servo_feedback_.error_code = frame.data[7];
    
    return true;
}
