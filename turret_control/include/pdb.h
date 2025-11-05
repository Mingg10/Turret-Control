#ifndef PDB_H
#define PDB_H

#include <cstdint>
#include <vector>
#include "pi3hat.h"  // use your provided pi3hat header

// Structure to hold power distribution board measurements.
struct PDBData {
    float voltage;  // in volts
    float current;  // in amperes
    float energy;   // in Wh (or your chosen unit)
};

class PDB {
public:
    // Constructor:
    //   pdb_id: the 16-bit CAN identifier (moteus-style) used by the power distribution board.
    //           For example, 0x8070 (if source=0x80 and destination=0x70).
    //   pi3hat: pointer to an initialized Pi3Hat interface.
    PDB(uint32_t pdb_id, mjbots::pi3hat::Pi3Hat* pi3hat);

    // Polls the board and fills out the PDBData structure.
    // Returns true if a valid frame is received.
    bool readData(PDBData& data);

private:
    uint32_t pdb_id_;  // Expected CAN id for the PDB.
    mjbots::pi3hat::Pi3Hat* pi3hat_;

    // Persistent buffers for CAN communication.
    std::vector<mjbots::pi3hat::CanFrame> tx_can_;
    std::vector<mjbots::pi3hat::CanFrame> rx_can_;

    // Unpacks a received CAN-FD frame into PDBData.
    bool unpackData(const mjbots::pi3hat::CanFrame& frame, PDBData& data);
};

#endif // PDB_H
