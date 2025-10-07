#include "choose_app.h"
#ifdef BUILD_SNIFFER_APP

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define DEBUG_MODULE "SNIFFER"

#include "common.h"
#include "debug.h"
#include "p2p_interface.h"
#include "param_log_interface.h"
#include "settings.h"

static xTimerHandle broadcastTimer;
static uint32_t endBroadcastTime = 0;

static void broadcastData(xTimerHandle timer)
{
    uint32_t nowMs = T2M(xTaskGetTickCount());

    copter_full_state_t fullState;

    fullState.id = 0; // The sniffer
    fullState.state = STATE_SNIFFING;
    fullState.battery_voltage = 0;
    fullState.timestamp = nowMs;
    fullState.position.x = 0.0f;
    fullState.position.y = 0.0f;
    fullState.position.z = 0.0f;

    broadcastToPeers(&fullState, nowMs);

    for (uint8_t i = 0; i < MAX_ADDRESS; i++) {

        copter_full_state_t peer = getPeerState(i);

        if (peer.fullState.swarmalatorParamsCounter != swarmalatorParamsCounter[i]) {
            // Send packet to update the swarmalator params on the peer

            swarmalator_params_t swarmalatorParams;
            swarmalatorParams.K = Ks[i];
            swarmalatorParams.J = Js[i];
            swarmalatorParams.A = As[i];
            swarmalatorParams.B = Bs[i];
            swarmalatorParams.naturalFrequency = naturalFrequencies[i];
            swarmalatorParams.startingPhase = startingPhases[i];

            broadcastSwarmalatorParams(i, &swarmalatorParams[i]);
        }
    }
}

static uint8_t isExperimentRunningVal = 0;
static void isExperimentRunningCB(void)
{
    setIsRunning(isExperimentRunningVal);

    endBroadcastTime = T2M(xTaskGetTickCount()) + 1000; // Broadcast for 1 second
}

static float Ks[MAX_ADDRESS];
static float Js[MAX_ADDRESS];
static float As[MAX_ADDRESS];
static float Bs[MAX_ADDRESS];
static float naturalFrequencies[MAX_ADDRESS];
static float startingPhases[MAX_ADDRESS];
static uint8_t swarmalatorParamsCounter[MAX_ADDRESS];

void swarmalatorParamsCB(uint8_t id)
{
    swarmalatorParamsCounter[id]++;
}

void appMain()
{
    DEBUG_PRINT("SWARMALATOR SNIFFER running\n");

    initP2P();
    initPeerStates();

    broadcastTimer = xTimerCreate("SendCommandTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(broadcastTimer, 20);

    while (1) {
        vTaskDelay(M2T(SNIFFER_PRINT_PERIOD_MS));
        printPeers();

        DEBUG_PRINT("Experiment running: %u\n", isExperimentRunningVal);
    }
}

#define SWARMALATOR_AGENT_PARAMS(AGENT_ID) \
    void swarmalatorParamsCB_##AGENT_ID(void) { \
        swarmalatorParamsCB(AGENT_ID); \
    } \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_K, &Ks[AGENT_ID], swarmalatorParamsCB_##AGENT_ID); \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_J, &Js[AGENT_ID], swarmalatorParamsCB_##AGENT_ID); \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_A, &As[AGENT_ID], swarmalatorParamsCB_##AGENT_ID); \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_B, &Bs[AGENT_ID], swarmalatorParamsCB_##AGENT_ID); \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_naturalFrequency, &naturalFrequencies[AGENT_ID], swarmalatorParamsCB_##AGENT_ID); \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_phase, &startingPhases[AGENT_ID], swarmalatorParamsCB_##AGENT_ID);

PARAM_GROUP_START(app)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, isExperimentRunning, &isExperimentRunningVal, &isExperimentRunningCB)
SWARMALATOR_AGENT_PARAMS(0)
SWARMALATOR_AGENT_PARAMS(1)
SWARMALATOR_AGENT_PARAMS(2)
SWARMALATOR_AGENT_PARAMS(3)
SWARMALATOR_AGENT_PARAMS(4)
SWARMALATOR_AGENT_PARAMS(5)
SWARMALATOR_AGENT_PARAMS(6)
SWARMALATOR_AGENT_PARAMS(7)
SWARMALATOR_AGENT_PARAMS(8)
SWARMALATOR_AGENT_PARAMS(9)
PARAM_GROUP_STOP(app)

#endif // BUILD_SNIFFER_APP