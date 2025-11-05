#ifndef CUBEMARS_CONTROL_H
#define CUBEMARS_CONTROL_H

#include <vector>
#include "pi3hat.h"  // use your provided pi3hat header

// This class implements CubeMars motor control over the pi3hat CAN interface in MIT power mode,
// using synchronous command transmission (one command at a time) and standard CAN frame format.
class CubemarsControl {
public:
    // Constructor:
    //   motor_id: the CAN ID of the motor
    //   can_bus: (for compatibility; we force use of JC5)
    //   pi3hat: pointer to the initialized Pi3Hat interface.
    CubemarsControl(int motor_id, int can_bus, mjbots::pi3hat::Pi3Hat* pi3hat);

    // MIT Mode command functions (synchronous)
    void sendCommandMITMode(float pos, float vel, float kp, float kd, float torq);
    void enterMITMode();
    void exitMITMode();
    void zeroMotor();

    // Servo Mode command functions (synchronous)
    // Each function builds a servo command using extended CAN frame format
    void sendDutyCycleMode(float duty_cycle);
    void sendCurrentLoopMode(float current_amps);
    void sendCurrentBrakeMode(float brake_current_amps);
    void sendSpeedLoopMode(float speed_erpm);
    void sendPositionLoopMode(float position_degrees);
    void setOriginMode(uint8_t origin_type = 0);  // 0=temporary, 1=permanent
    void sendPositionSpeedLoopMode(float position_degrees, int16_t speed_erpm, int16_t acceleration);

    // Getters for feedback (works for both MIT and Servo modes)
    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;
    float getTemperature() const;
    int getErrorFlag() const;

    // Servo mode specific feedback (if using servo mode with periodic feedback)
    struct ServoFeedback {
        float position_degrees = 0.0f;      // -3200° to 3200°
        float speed_erpm = 0.0f;             // -320000 to 320000 ERPM
        float current_amps = 0.0f;           // -60 to 60A
        float temperature_celsius = 0.0f;    // -20°C to 127°C
        uint8_t error_code = 0;              // 0=no fault, 1=over-temp, 2=over-current, etc.
    };
    
    ServoFeedback getServoFeedback() const { return servo_feedback_; }

private:
    // Helper functions for fixed-point conversion.
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);

    // Unpacks a received CAN frame.
    bool unpackReply(const mjbots::pi3hat::CanFrame &frame);

    // Parameter limits matching the manual for MIT mode.
    static constexpr float P_MIN = -95.5f;
    static constexpr float P_MAX = +95.5f;
    static constexpr float V_MIN = -30.0f;
    static constexpr float V_MAX = +30.0f;
    static constexpr float T_MIN = -18.0f;
    static constexpr float T_MAX = +18.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    // Servo mode control modes (from CubeMars manual)
    enum class ServoMode : uint32_t {
        DUTY_CYCLE = 0,           // Duty cycle voltage control
        CURRENT_LOOP = 1,         // Iq current control (torque)
        CURRENT_BRAKE = 2,        // Braking current control
        SPEED_LOOP = 3,           // Speed control
        POSITION_LOOP = 4,        // Position control
        SET_ORIGIN = 5,           // Set zero reference
        POSITION_SPEED_LOOP = 6   // Position with speed/acceleration
    };

    int motor_id_;
    int can_bus_;  // Provided for API compatibility (but we force JC5 in all commands)
    mjbots::pi3hat::Pi3Hat* pi3hat_;

    struct MotorData {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;
        float temperature = 0.0f;
        int error_flag = 0;
    } motor_data_;

    // Servo mode feedback data
    ServoFeedback servo_feedback_;
    
    // Helper functions for servo mode
    void sendServoCommand(ServoMode mode, const uint8_t* data, size_t data_len);
    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index);
    void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t* index);
    bool unpackServoReply(const mjbots::pi3hat::CanFrame &frame);

    // Persistent buffers for a single command (we use a vector of 5 slots, as in the previous design).
    std::vector<mjbots::pi3hat::CanFrame> tx_can_;
    std::vector<mjbots::pi3hat::CanFrame> rx_can_;
};

#endif
