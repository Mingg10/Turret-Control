#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <limits>
#include "cubemars_control.h"
#include "pigpio.h"
#include <memory>

std::atomic<bool> running(true);
std::atomic<bool> stop_motion(false);

void signal_handler(int)
{
    running = false;
    stop_motion = true;
}

// Spool parameters
const float SPOOL_DIAMETER_MM = 26.0f;  // 26mm spool diameter
const float SPOOL_RADIUS_M = (SPOOL_DIAMETER_MM / 2.0f) / 1000.0f;  // Convert to meters
const float PI = 3.14159265359f;
const float TWO_PI = 2.0f * PI;
const float GEAR_RATIO = 6.0f;  // AK60-6 has 6:1 gear reduction

struct AngleUnwrapper {
    float previous_raw = std::numeric_limits<float>::quiet_NaN();
    float previous_unwrapped = 0.0f;
    float max_step = PI;  // Largest believable change between samples

    void reset(float reference_raw) {
        previous_raw = reference_raw;
        previous_unwrapped = reference_raw;
    }

    float unwrap(float current_raw) {
        if (!std::isfinite(previous_raw)) {
            reset(current_raw);
            return previous_unwrapped;
        }

        float delta = current_raw - previous_raw;

        if (delta > PI) {
            delta -= TWO_PI;
        } else if (delta < -PI) {
            delta += TWO_PI;
        }

        if (std::abs(delta) > max_step) {
            // Ignore sudden spikes but keep the encoder's raw state
            previous_raw = current_raw;
            return previous_unwrapped;
        }

        previous_unwrapped += delta;
        previous_raw = current_raw;
        return previous_unwrapped;
    }
};

// IMPORTANT: MIT mode behavior for AK60-6:
// - Velocity command: Applied to OUTPUT shaft (0.3 rad/s moves output shaft at 0.3 rad/s)
// - Position feedback: Reports INPUT/MOTOR shaft position (encoder is on motor shaft)
// Therefore: position readings must be divided by GEAR_RATIO to get output shaft position

// Function to convert wire length (meters) to output shaft rotation (radians)
float wireLengthToRadians(float length_m) {
    // Circumference = 2 * PI * radius
    // Output shaft rotations needed = length / circumference
    // Output shaft radians = rotations * 2 * PI
    float circumference = 2.0f * PI * SPOOL_RADIUS_M;
    float output_rotations = length_m / circumference;
    float output_radians = output_rotations * 2.0f * PI;
    return output_radians;
}

// Function to convert output shaft rotation (radians) to wire length (meters)
float outputRadiansToWireLength(float output_radians) {
    // Output shaft rotations = radians / (2 * PI)
    // Wire length = rotations * circumference
    float output_rotations = output_radians / (2.0f * PI);
    float circumference = 2.0f * PI * SPOOL_RADIUS_M;
    float length_m = output_rotations * circumference;
    return length_m;
}

// Function to convert motor shaft encoder reading to wire length
float motorEncoderToWireLength(float motor_radians) {
    // Motor shaft radians -> output shaft radians (divide by gear ratio)
    // Then convert output radians to wire length
    float output_radians = motor_radians / GEAR_RATIO;
    return outputRadiansToWireLength(output_radians);
}

// Function to get user input for desired wire length
bool getUserWireLength(float current_length_cm, float& desired_length_cm) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Current wire length: " << current_length_cm << " cm" << std::endl;
    std::cout << "(F) Enter desired wire length in cm (or 'q' to quit): ";

    std::string input;
    std::cin >> input;

    if (input == "q" || input == "Q") {
        return false;
    }

    try {
        desired_length_cm = std::stof(input);
        return true;
    } catch (...) {
        std::cout << "Invalid input. Please enter a number." << std::endl;
        return getUserWireLength(current_length_cm, desired_length_cm);
    }
}

// Velocity-controlled wire actuation function
bool actuateWireToLength(CubemarsControl* motor,
                        AngleUnwrapper& encoder_unwrapper,
                        float zero_position_unwrapped,
                        float current_length_cm,
                        float desired_length_cm,
                        float velocity_rad_per_sec,
                        float& final_length_cm) {

    const float POSITION_TOLERANCE_CM = 0.2f;  // 2mm tolerance
    const float MAX_RUNTIME_SEC = 30.0f;
    const float SLOWDOWN_DISTANCE_CM = 2.0f;  // Start slowing down 2cm before target

    // Calculate length change needed
    float length_change_cm = desired_length_cm - current_length_cm;
    float length_change_m = length_change_cm / 100.0f;

    std::cout << "\n--- Wire Actuation ---" << std::endl;
    std::cout << "Current length: " << current_length_cm << " cm" << std::endl;
    std::cout << "Desired length: " << desired_length_cm << " cm" << std::endl;
    std::cout << "Change needed: " << length_change_cm << " cm";

    if (length_change_cm > 0) {
        std::cout << " (EXTENDING)" << std::endl;
    } else if (length_change_cm < 0) {
        std::cout << " (RETRACTING)" << std::endl;
    } else {
        std::cout << " (No change needed)" << std::endl;
        final_length_cm = current_length_cm;
        return true;
    }

    // Calculate target position (output shaft radians needed)
    float target_output_radians = wireLengthToRadians(std::abs(length_change_m));
    if (length_change_cm < 0) {
        target_output_radians = -target_output_radians;  // Negative for retraction
    }

    // Motor encoder will need to read 6x this value
    float target_motor_radians = target_output_radians * GEAR_RATIO;

    // Determine velocity direction
    float commanded_velocity = (length_change_cm > 0) ? velocity_rad_per_sec : -velocity_rad_per_sec;

    std::cout << "Target output shaft rotation: " << target_output_radians << " rad ("
              << target_output_radians * 180.0f / PI << " deg)" << std::endl;
    std::cout << "Expected motor encoder change: " << target_motor_radians << " rad ("
              << target_motor_radians * 180.0f / PI << " deg)" << std::endl;
    std::cout << "Velocity (output shaft): " << commanded_velocity << " rad/s" << std::endl;
    std::cout << "Control: Velocity mode with Kp=0, Kd=1.0" << std::endl;

    // MIT mode control gains for velocity control
    float kp = 0.0f;    // No position feedback
    float kd = 1.0f;    // Damping for velocity control
    float feedforward_torque = 0.0f;

    auto start_time = std::chrono::steady_clock::now();
    stop_motion = false;

    std::cout << "\nActuating..." << std::endl;

    float start_position = 0.0f;
    bool first_iteration = true;

    while (running.load() && !stop_motion.load()) {
        float raw_position = motor->getPosition();
        float current_position = encoder_unwrapper.unwrap(raw_position);

        if (first_iteration) {
            start_position = current_position;
            first_iteration = false;
        }

        // Calculate current wire length from zero position
        // (Position is motor shaft, convert to wire length)
        float current_wire_length_m = motorEncoderToWireLength(current_position - zero_position_unwrapped);
        float current_wire_length_cm = current_wire_length_m * 100.0f;

        // Check remaining distance and adjust velocity for smooth approach
        float remaining_cm = desired_length_cm - current_wire_length_cm;
        float active_velocity = commanded_velocity;

        // Slow down as we approach the target
        if (std::abs(remaining_cm) < SLOWDOWN_DISTANCE_CM) {
            float slowdown_factor = std::abs(remaining_cm) / SLOWDOWN_DISTANCE_CM;
            slowdown_factor = std::max(0.2f, slowdown_factor);  // Minimum 20% speed
            active_velocity = commanded_velocity * slowdown_factor;
        }

        // Send velocity command (applied to output shaft)
        motor->sendCommandMITMode(0.0f, active_velocity, kp, kd, 0);

        // Read back motor state
        float current_velocity = motor->getVelocity();
        float current_torque = motor->getTorque();

        // Calculate traveled distance from start of this motion
        // (Both positions are motor shaft readings)
        float traveled_motor_radians = current_position - start_position;
        float traveled_length_m = motorEncoderToWireLength(traveled_motor_radians);
        float traveled_length_cm = traveled_length_m * 100.0f;

        // Display status
        std::cout << "\rWire: " << current_wire_length_cm << " cm"
                  << " | Travel: " << traveled_length_cm << " cm"
                  << " | Target: " << length_change_cm << " cm"
                  << " | Remaining: " << remaining_cm << " cm"
                  << " | Vel: " << current_velocity << " rad/s"
                  << " | Torque: " << current_torque << " Nm"
                  << "        " << std::flush;

        // Check if target reached
        if (std::abs(remaining_cm) < POSITION_TOLERANCE_CM) {
            std::cout << "\n\nTarget reached!" << std::endl;
            final_length_cm = current_wire_length_cm;

            // Gradual stop: reduce velocity to zero over 20 iterations
            for (int i = 0; i < 20; i++) {
                float ramp_factor = 1.0f - (static_cast<float>(i) / 20.0f);
                motor->sendCommandMITMode(0.0f, 0.0f, 0.0f, 3.0f, 0.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Final hold with moderate gains
            float hold_pos_raw = motor->getPosition();
            float hold_pos = encoder_unwrapper.unwrap(hold_pos_raw);  // Keep state continuous
            for (int i = 0; i < 10; i++) {
                motor->sendCommandMITMode(hold_pos, 0.0f, 10.0f, 3.0f, 0.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return true;
        }

        // Check timeout
        auto elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > MAX_RUNTIME_SEC) {
            std::cout << "\n\nTimeout reached!" << std::endl;
            final_length_cm = current_wire_length_cm;

            // Gradual stop with damping only
            for (int i = 0; i < 20; i++) {
                motor->sendCommandMITMode(0.0f, 0.0f, 0.0f, 3.0f, 0.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Hold position with moderate gains
            float hold_position_raw = motor->getPosition();
            float hold_position = encoder_unwrapper.unwrap(hold_position_raw);
            for (int i = 0; i < 10; i++) {
                motor->sendCommandMITMode(hold_position, 0.0f, 10.0f, 3.0f, 0.0f);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return false;
        }
    }

    // Emergency stop if interrupted
    std::cout << "\n\nMotion interrupted!" << std::endl;

    // Gradual stop with damping
    for (int i = 0; i < 20; i++) {
        motor->sendCommandMITMode(0.0f, 0.0f, 0.0f, 3.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Hold position with moderate gains
    float hold_position_raw = motor->getPosition();
    float hold_position = encoder_unwrapper.unwrap(hold_position_raw);
    for (int i = 0; i < 10; i++) {
        motor->sendCommandMITMode(hold_position, 0.0f, 10.0f, 3.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Update final length
    float current_position_raw = motor->getPosition();
    float current_position = encoder_unwrapper.unwrap(current_position_raw);
    float current_wire_length_m = motorEncoderToWireLength(current_position - zero_position_unwrapped);
    final_length_cm = current_wire_length_m * 100.0f;

    return false;
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return -1;
    }
    std::signal(SIGINT, signal_handler);

    try {
        // Pi3Hat configuration
        mjbots::pi3hat::Pi3Hat::Configuration config;

        // Motor channel: JC5 (classical CAN, 1 Mbps)
        config.can[4].slow_bitrate = 1000000;
        config.can[4].fdcan_frame = false;
        config.can[4].bitrate_switch = false;

        // Disable unused channels
        config.can[0].slow_bitrate = 0;
        config.can[1].slow_bitrate = 0;
        config.can[2].slow_bitrate = 0;
        config.can[3].slow_bitrate = 0;

        auto pi3hat = std::make_unique<mjbots::pi3hat::Pi3Hat>(config);

        // Create motor control object on JC5 (motor ID = 10)
        auto motor = std::make_unique<CubemarsControl>(10, 0, pi3hat.get());
        AngleUnwrapper encoder_unwrapper;

        std::cout << "========================================" << std::endl;
        std::cout << "   Velocity-Controlled Wire Spool" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Spool diameter: " << SPOOL_DIAMETER_MM << " mm" << std::endl;
        std::cout << "Spool radius: " << SPOOL_RADIUS_M * 1000.0f << " mm" << std::endl;
        std::cout << "Spool circumference: " << 2.0f * PI * SPOOL_RADIUS_M * 1000.0f << " mm" << std::endl;

        // Enter MIT mode
        std::cout << "\nEntering MIT mode..." << std::endl;
        motor->enterMITMode();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Zero the motor at current position (this is our reference point)
        std::cout << "Zeroing motor position (this will be the reference point)..." << std::endl;
        motor->zeroMotor();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Get zero position (motor shaft encoder reading)
        motor->sendCommandMITMode(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        float zero_position_raw = motor->getPosition();
        encoder_unwrapper.reset(zero_position_raw);
        float zero_position_unwrapped = encoder_unwrapper.unwrap(zero_position_raw);
        std::cout << "Zero position set at: " << zero_position_raw << " radians (motor shaft raw)" << std::endl;

        // Configure velocity for wire actuation (rad/s at OUTPUT shaft)
        // MIT mode uses output shaft coordinates (gear ratio already handled internally)
        float wire_velocity_rad_per_sec = 0.3f;  // Output shaft velocity
        std::cout << "\nOutput shaft velocity: " << wire_velocity_rad_per_sec << " rad/s" << std::endl;
        std::cout << "Wire speed at spool: "
                  << (wire_velocity_rad_per_sec * SPOOL_RADIUS_M * 100.0f) << " cm/s" << std::endl;

        // Track current wire length
        float current_wire_length_cm = 0.0f;  // Start at zero (just zeroed)

        std::cout << "\n========================================" << std::endl;
        std::cout << "Ready for wire control!" << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;

        // Main control loop
        while (running.load()) {
            float desired_length_cm = 0.0f;

            // Get user input for desired wire length
            if (!getUserWireLength(current_wire_length_cm, desired_length_cm)) {
                std::cout << "Exiting..." << std::endl;
                break;
            }

            // Actuate to desired length
            float final_length_cm = current_wire_length_cm;
            bool success = actuateWireToLength(motor.get(),
                                              encoder_unwrapper,
                                              zero_position_unwrapped,
                                              current_wire_length_cm,
                                              desired_length_cm,
                                              wire_velocity_rad_per_sec,
                                              final_length_cm);

            // Update tracked length
            current_wire_length_cm = final_length_cm;

            if (success) {
                std::cout << "Actuation complete." << std::endl;
                std::cout << "Final wire length: " << current_wire_length_cm << " cm" << std::endl;
            } else {
                std::cout << "Actuation incomplete or interrupted." << std::endl;
                std::cout << "Current wire length: " << current_wire_length_cm << " cm" << std::endl;
            }

            // If motion was interrupted by Ctrl+C, exit loop
            if (!running.load()) {
                break;
            }
        }

        // Exit MIT mode
        std::cout << "\nExiting MIT mode..." << std::endl;
        motor->exitMITMode();

        std::cout << "Program terminated." << std::endl;
        std::cout << "Final wire length from zero: " << current_wire_length_cm << " cm" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        gpioTerminate();
        return -1;
    }

    gpioTerminate();
    return 0;
}
