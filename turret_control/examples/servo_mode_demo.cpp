#include "cubemars_control.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    try {
        // Initialize Pi3Hat
        mjbots::pi3hat::Pi3Hat::Configuration pi3hat_config;
        mjbots::pi3hat::Pi3Hat pi3hat(pi3hat_config);
        
        // Create motor controller (motor ID 1, CAN bus 5)
        CubemarsControl motor(1, 5, &pi3hat);
        
        std::cout << "=== CubeMars AK60-6 Servo Mode Demo ===" << std::endl;
        std::cout << "This demo shows all 6 servo control modes" << std::endl;
        
        // 1. Duty Cycle Mode - 20% duty cycle
        std::cout << "\n1. Duty Cycle Mode (20% duty cycle)" << std::endl;
        motor.sendDutyCycleMode(0.2f);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        auto servo_feedback = motor.getServoFeedback();
        std::cout << "Position: " << servo_feedback.position_degrees << "°" << std::endl;
        std::cout << "Speed: " << servo_feedback.speed_erpm << " ERPM" << std::endl;
        std::cout << "Current: " << servo_feedback.current_amps << " A" << std::endl;
        
        // 2. Current Loop Mode - 2A torque
        std::cout << "\n2. Current Loop Mode (2A torque)" << std::endl;
        motor.sendCurrentLoopMode(2.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 3. Speed Loop Mode - 1000 ERPM
        std::cout << "\n3. Speed Loop Mode (1000 ERPM)" << std::endl;
        motor.sendSpeedLoopMode(1000.0f);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        servo_feedback = motor.getServoFeedback();
        std::cout << "Speed: " << servo_feedback.speed_erpm << " ERPM" << std::endl;
        
        // 4. Position Loop Mode - 90 degrees
        std::cout << "\n4. Position Loop Mode (90°)" << std::endl;
        motor.sendPositionLoopMode(90.0f);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        servo_feedback = motor.getServoFeedback();
        std::cout << "Position: " << servo_feedback.position_degrees << "°" << std::endl;
        
        // 5. Set Origin - Set current position as zero
        std::cout << "\n5. Set Origin Mode (temporary zero point)" << std::endl;
        motor.setOriginMode(0);  // 0 = temporary origin
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 6. Position-Speed Loop Mode - Move to 180° with controlled speed/acceleration
        std::cout << "\n6. Position-Speed Loop Mode (180°, 5000 ERPM, 30000 accel)" << std::endl;
        motor.sendPositionSpeedLoopMode(180.0f, 5000, 30000);
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
        servo_feedback = motor.getServoFeedback();
        std::cout << "Final Position: " << servo_feedback.position_degrees << "°" << std::endl;
        std::cout << "Final Speed: " << servo_feedback.speed_erpm << " ERPM" << std::endl;
        
        // 7. Current Brake Mode - Hold position with 5A brake current
        std::cout << "\n7. Current Brake Mode (5A brake current)" << std::endl;
        motor.sendCurrentBrakeMode(5.0f);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        servo_feedback = motor.getServoFeedback();
        std::cout << "Brake Current: " << servo_feedback.current_amps << " A" << std::endl;
        std::cout << "Temperature: " << servo_feedback.temperature_celsius << "°C" << std::endl;
        std::cout << "Error Code: " << static_cast<int>(servo_feedback.error_code) << std::endl;
        
        // Stop motor
        std::cout << "\nStopping motor..." << std::endl;
        motor.sendCurrentLoopMode(0.0f);
        
        std::cout << "\n=== Demo Complete ===" << std::endl;
        std::cout << "All 6 servo modes have been demonstrated successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}