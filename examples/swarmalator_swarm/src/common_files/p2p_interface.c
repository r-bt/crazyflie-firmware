#include "FreeRTOS.h"
#include "debug.h"
#include "log.h"
#include "task.h"

#include "common.h"
#include "p2p_interface.h"
#include "settings.h"

#define THE_MAGIC_NUMBER 0x12345678
#define P2P_PORT 5 // Crazyflie allows PORT to be specified to differentiate between different types of packets

// State of peer copters
copter_full_state_t copters[MAX_ADDRESS];

// Keeps track of the number of packets sent
static uint8_t counter = 0;

static void p2pcallbackHandler(P2PPacket* p)
{
    if (p->port != P2P_PORT) {
        DEBUG_PRINT("Wrong port %u\n", p->port);
        return;
    }

    static copter_message_t rxMessage;

    uint32_t nowMs = T2M(xTaskGetTickCount());

    // Copy the received data into the rxMessage struct
    memcpy(&rxMessage, p->data, sizeof(rxMessage));

    // Check if the magic number is correct
    if (rxMessage.magicNumber != THE_MAGIC_NUMBER) {
        DEBUG_PRINT("Wrong magic number %lu from %u\n", rxMessage.magicNumber, rxMessage.fullState.id);
        return;
    }

    // LOG who we received a message from
    uint8_t received_id = rxMessage.fullState.id;

    if (received_id >= MAX_ADDRESS) {
        DEBUG_PRINT("Invalid ID %u\n", received_id);
        return;
    }

    // Save the copter state
    memcpy(&copters[received_id], &rxMessage.fullState, sizeof(copter_full_state_t));
    copters[received_id].timestamp = nowMs;

    // If not sniffer, record the position with the peer localization service
    if (received_id > 0) {
        enum State state = rxMessage.fullState.state;

        if (state == STATE_TAKING_OFF || state == STATE_HOVERING || state == STATE_EXECUTING_SWARMALATOR || state == STATE_PREPARING_FOR_LAND || state == STATE_LANDING) {

            positionMeasurement_t pos_measurement;
            memcpy(&pos_measurement.pos, &rxMessage.fullState.position, sizeof(Position));

            pos_measurement.source = MeasurementSourceLighthouse;
            pos_measurement.stdDev = 0.01f;

            peerLocalizationTellPosition(received_id, &pos_measurement);
        }
    }
}

void initP2P()
{
    // Register callback handler for P2P packets
    p2pRegisterCB(p2pcallbackHandler);
}

void broadcastToPeers(const copter_full_state_t* state, const uint32_t nowMs)
{
    static P2PPacket packet;
    static copter_message_t txMessage;

    memcpy(&txMessage.fullState, state, sizeof(copter_full_state_t));
    txMessage.fullState.counter = counter;

    txMessage.magicNumber = THE_MAGIC_NUMBER;

    packet.port = P2P_PORT;
    memcpy(packet.data, &txMessage, sizeof(txMessage));

    packet.size = sizeof(txMessage);

    // Broadcast the packet (TODO: Does this handle collisions?)
    radiolinkSendP2PPacketBroadcast(&packet);

    counter++;
}

uint8_t compressVoltage(float voltage)
{
    if (voltage < VOLTAGE_MIN) {
        return 0;
    }

    if (voltage > VOLTAGE_MAX) {
        return 255;
    }

    return (uint8_t)((voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 255);
}

void printPeers(void)
{
    for (int i = 0; i < MAX_ADDRESS; i++) {
        if (copters[i].state != STATE_UNKNOWN) {
            if (!peerLocalizationIsIDActive(i)) {
                DEBUG_PRINT("Copter %d is not active\n", i);
            } else {
                peerLocalizationOtherPosition_t* pos = peerLocalizationGetPositionByID(i);
                DEBUG_PRINT("Copter %d : %.2f , %.2f , %.2f --> %d with latest counter %d \n", i, (double)pos->pos.x, (double)pos->pos.y, (double)pos->pos.z, copters[i].state, copters[i].counter);
            }
        }
    }
}

void initPeerStates()
{
    for (int i = 0; i < MAX_ADDRESS; i++) {
        copters[i].state = STATE_UNKNOWN;
    }
}