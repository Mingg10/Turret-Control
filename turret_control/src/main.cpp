#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include "cubemars_control.h"
#include "pdb.h"
#include "pigpio.h"
#include <memory>

std::atomic<bool> running(true);

void signal_handler(int)
{
    running = false;
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return -1;
    }
    std::signal(SIGINT, signal_handler);

    try {
        // Pi3Hat configuration for multiple channels:
        mjbots::pi3hat::Pi3Hat::Configuration config;
        // For JC1 (config.can[0]), use CAN FD for the PDB.
        config.can[0].slow_bitrate = 1000000;  // 1 Mbps arbitration
        config.can[0].fast_bitrate = 5000000;    // 5 Mbps data phase (CAN FD)
        config.can[0].fdcan_frame = true;
        config.can[0].bitrate_switch = true;
        
        // Disable (or leave at zero) unused channels if desired.
        config.can[1].slow_bitrate = 0;
        config.can[2].slow_bitrate = 0;
        config.can[3].slow_bitrate = 0;
        
        // Motor channel: JC5 (config.can[4]) remains unchanged (classical CAN, 1 Mbps)
        config.can[4].slow_bitrate = 1000000;
        config.can[4].fdcan_frame = false;
        config.can[4].bitrate_switch = false;
        
        auto pi3hat = std::make_unique<mjbots::pi3hat::Pi3Hat>(config);

        

        // Create motor control object on JC5 (motor code working as is)
        auto motor = std::make_unique<CubemarsControl>(10, 0, pi3hat.get());
        // Create PDB object on JC1, using the extended CAN id appropriate for the power board.
        // For example, assume the PDB uses extended CAN id 0x18FF50E5 (adjust to your board!)
        // PDB pdb(0x8070, pi3hat.get());
        
        // Put the motor into MIT mode (unchanged)
        // motor->enterMITMode();
        
        // while (running.load()) {
        //     // Send motor command to motor1 (unchanged, on JC5).
        //     motor->sendCommandMITMode(0.0f, 3.0f, 0.0f, 1.0f, 0.0f);
            
        //     // Poll the PDB on JC1 using CAN FD.
        //     // PDBData powerData;
        //     // if (pdb.readData(powerData)) {
        //     //     std::cout << "PDB Voltage: " << powerData.voltage 
        //     //               << " V, Current: " << powerData.current 
        //     //               << " A, Energy: " << powerData.energy << " Wh" 
        //     //               << std::endl;
        //     // }
            
        //     std::cout << "Motor1 - Position: " << motor->getPosition() 
        //               << ", Velocity: " << motor->getVelocity() 
        //               << ", Torque: " << motor->getTorque() << std::endl;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }

        // // Clean up motor and PDB (exit MIT mode, etc.)
        // motor->exitMITMode();


        while (running.load())
        {
            // 3. Speed Loop Mode - 1000 ERPM
            std::cout << "\n3. Speed Loop Mode (1000 ERPM)" << std::endl;
            motor->sendSpeedLoopMode(5000.0f);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            auto servo_feedback = motor->getServoFeedback();
            std::cout << "Speed: " << servo_feedback.speed_erpm << " ERPM" << std::endl;
        
        }
        

        

        std::cout << "Test stopped." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        gpioTerminate();
        return -1;
    }
    gpioTerminate();
    return 0;
}
