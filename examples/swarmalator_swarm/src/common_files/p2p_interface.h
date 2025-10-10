#ifndef P2P_INTERFACE_H
#define P2P_INTERFACE_H

#include <string.h>

#include "peer_localization.h"
#include "positions.h"
#include "radiolink.h"
#include "common.h"

typedef struct {
    // State
    uint8_t id;
    uint8_t counter; // TODO: What does this do?
    uint8_t state;
    uint8_t battery_voltage; // Normalized to 0-255 (0-3.3V)
    uint32_t timestamp;
    Position position;
    float phase;
    uint32_t swarmalatorParamsVersion;
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
    uint32_t magicNumber;
    uint8_t id;
    uint32_t swarmalatorParamsVersion;
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
 * Broadcast swarmalator params for a specific drone
 * 
 * Note broadcasts to all drones but drones check if id in packet matches their id
 */
void broadcastSwarmalatorParams(uint8_t id, uint32_t swarmalatorParamsVersion, const swarmalator_params_t* swarmalatorParams);

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

uint32_t getSwarmalatorsParamsVersion(void);

/**
 * Get's the state of a peer
 *
 * @param id The ID of the peer
 * @return The state of the peer
 */
copter_full_state_t getPeerState(uint8_t id);

/**
 * Get's the swarmalator parameters of this copter
 *
 * @return Pointer to the swarmalator parameters
 */
swarmalator_params_t* getSwarmalatorParams(void);

#endif // P2P_INTERFACE_H