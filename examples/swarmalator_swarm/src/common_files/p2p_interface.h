#ifndef P2P_INTERFACE_H
#define P2P_INTERFACE_H

#include <string.h>

#include "peer_localization.h"
#include "positions.h"
#include "radiolink.h"

typedef struct {
    // State
    uint8_t id;
    uint8_t counter; // TODO: What does this do?
    uint8_t state;
    uint8_t battery_voltage; // Normalized to 0-255 (0-3.3V)
    uint32_t timestamp;
    Position position;
    float phase;
    uint8_t swarmalatorParamsCounter;
} copter_full_state_t;

typedef struct {
    copter_full_state_t fullState;

    // Higher level control
    uint32_t controlDataVersion;
    uint8_t isControlDataValid;
    uint32_t magicNumber;

    // Control parameters
    uint8_t isRunning; // 0 = experiment not running, 1 = experiment running
} copter_message_t;

typedef struct {
    unit32_t magicNumber;
    uint8_t id;
    swarmalator_params_t swarmalatorParams;

} swarmalator_params_message_t;

/**
 * Initialize the P2P interface
 *
 * This function should be called once at the beginning of the program
 */
void initP2P();

/**
 * Broadcast the Crazyflie's state to all other Crazyflies
 *
 * @param state The state of the Crazyflie
 * @param nowMs The current time in ms
 */
void broadcastToPeers(const copter_full_state_t* state, const uint32_t nowMs);

/**
 * Compress the voltage to a uint8_t value between 0 and 255
 *
 * @param voltage The voltage to compress
 * @return The compressed voltage
 */
uint8_t compressVoltage(float voltage);

/**
 * Print the state of all peers.
 *
 * Used by sniffer to communicate with GUI
 */
void printPeers(void);

/**
 * Init peer states
 *
 * Set all peer states to unknown
 */
void initPeerStates(void);

/**
 * Check if the experiment is running
 *
 * @return True if the experiment is running, false otherwise
 */
bool isExperimentRunning(void);

/**
 * Set the experiment running state
 */
void setIsRunning(uint8_t isRunningVal);

/**
 * Get's the state of a peer
 *
 * @param id The ID of the peer
 * @return The state of the peer
 */
copter_full_state_t getPeerState(uint8_t id);

#endif // P2P_INTERFACE_H