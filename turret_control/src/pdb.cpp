#include "pdb.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

static const size_t NUM_SLOTS = 5;

PDB::PDB(uint32_t pdb_id, mjbots::pi3hat::Pi3Hat* pi3hat)
    : pdb_id_(pdb_id), pi3hat_(pi3hat)
{
    tx_can_.resize(NUM_SLOTS);
    rx_can_.resize(NUM_SLOTS);
}

bool PDB::readData(PDBData& data) {
    // We will try two approaches.
    // First, send a poll command using the moteus "write registers" subframe.
    // Then, if no reply is received, try sending an empty TX frame (listen-only).
    
    // Approach A: Poll command payload.
    {
        tx_can_[0].id = pdb_id_;
        tx_can_[0].bus = 1;   // JC1 for PDB.
        tx_can_[0].size = 8;  // 8-byte payload.
        tx_can_[0].expect_reply = true;
        // Build payload per reference:
        // Byte 0: subframe type for write registers (0x00)
        // Byte 1: number of registers requested (0x03: voltage, current, energy)
        // Byte 2: starting register number (0x00)
        // Bytes 3-7: NOP padding (0x50)
        tx_can_[0].data[0] = 0x00;
        tx_can_[0].data[1] = 0x03;
        tx_can_[0].data[2] = 0x00;
        for (int i = 3; i < 8; i++) {
            tx_can_[0].data[i] = 0x50;
        }
        
        mjbots::pi3hat::Pi3Hat::Input input;
        input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
        input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
        // Use CAN1 (JC1) for polling.
        input.force_can_check = (1 << 1);
        input.timeout_ns = 10000000; // 10 ms timeout.
        
        const int maxAttempts = 10;
        int attempt = 0;
        while (attempt < maxAttempts) {
            auto output = pi3hat_->Cycle(input);
            for (size_t i = 0; i < output.rx_can_size; i++) {
                // For extended frames, mask with 0x1FFFFFFF.
                if ((rx_can_[i].id & 0x1FFFFFFF) == pdb_id_) {
                    if (unpackData(rx_can_[i], data)) {
                        return true;
                    }
                }
            }
            attempt++;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // Approach B: Listen-only.
    {
        tx_can_[0].id = pdb_id_;
        tx_can_[0].bus = 1;
        tx_can_[0].size = 0;  // No poll command; just listen.
        tx_can_[0].expect_reply = true;
        mjbots::pi3hat::Pi3Hat::Input input;
        input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_can_.data(), tx_can_.size());
        input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_can_.data(), rx_can_.size());
        input.force_can_check = (1 << 1);
        input.timeout_ns = 20000000; // 20 ms timeout.
        
        const int maxAttempts = 20;
        int attempt = 0;
        while (attempt < maxAttempts) {
            auto output = pi3hat_->Cycle(input);
            for (size_t i = 0; i < output.rx_can_size; i++) {
                if ((rx_can_[i].id & 0x1FFFFFFF) == pdb_id_) {
                    if (unpackData(rx_can_[i], data)) {
                        return true;
                    }
                }
            }
            attempt++;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    std::cerr << "PDB readData: Command timeout" << std::endl;
    return false;
}

bool PDB::unpackData(const mjbots::pi3hat::CanFrame& frame, PDBData& data) {
    // Verify that the frame has at least 8 bytes.
    if (frame.size < 8) {
        std::cerr << "Invalid frame size for PDB data" << std::endl;
        return false;
    }
    // According to the moteus / power_dist reference,
    // the data layout is:
    // Bytes 0-1: Voltage (uint16, big-endian), voltage = raw * 0.1
    // Bytes 2-3: Current (uint16, big-endian), current = raw * 0.01
    // Bytes 4-7: Energy (uint32, big-endian), energy = raw * 0.001
    uint16_t raw_voltage = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    uint16_t raw_current = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    uint32_t raw_energy  = (static_cast<uint32_t>(frame.data[4]) << 24) |
                           (static_cast<uint32_t>(frame.data[5]) << 16) |
                           (static_cast<uint32_t>(frame.data[6]) << 8)  |
                           frame.data[7];

    data.voltage = raw_voltage * 0.1f;   // Voltage in volts.
    data.current = raw_current * 0.01f;    // Current in amperes.
    data.energy  = raw_energy  * 0.001f;    // Energy in Wh.
    return true;
}
