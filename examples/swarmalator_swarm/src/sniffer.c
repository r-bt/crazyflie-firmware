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

static float Ks[MAX_ADDRESS];
static float Js[MAX_ADDRESS];
static float As[MAX_ADDRESS];
static float Bs[MAX_ADDRESS];
static float naturalFrequencies[MAX_ADDRESS];
static float startingPhases[MAX_ADDRESS];
static uint32_t swarmalatorParamsVersions[MAX_ADDRESS];

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

        if (peer.state == STATE_UNKNOWN) {
            continue;
        }

        // DEBUG_PRINT("Current swarmalator params for %u: K=%f, J=%f, A=%f, B=%f, f=%f, phase=%f (version=%lu)\n", i,
        //     (double)Ks[i], (double)Js[i], (double)As[i], (double)Bs[i],
        //     (double)naturalFrequencies[i], (double)startingPhases[i],
        //     swarmalatorParamsVersions[i]);

        if (peer.swarmalatorParamsVersion != swarmalatorParamsVersions[i]) {
            // Send packet to update the swarmalator params on the peer

            swarmalator_params_t swarmalatorParams;
            swarmalatorParams.K = Ks[i];
            swarmalatorParams.J = Js[i];
            swarmalatorParams.A = As[i];
            swarmalatorParams.B = Bs[i];
            swarmalatorParams.naturalFrequency = naturalFrequencies[i];
            swarmalatorParams.startingPhase = startingPhases[i];

            broadcastSwarmalatorParams(i, swarmalatorParamsVersions[i], &swarmalatorParams);

            DEBUG_PRINT("Sending swarmalator params to %u \n", i);
        }
    }
}

static uint8_t isExperimentRunningVal = 0;
static void isExperimentRunningCB(void)
{
    setIsRunning(isExperimentRunningVal);

    endBroadcastTime = T2M(xTaskGetTickCount()) + 1000; // Broadcast for 1 second
}


void swarmalatorParamsCB(uint8_t id)
{
    DEBUG_PRINT("Swarmalator params for %u changed\n", id);
    swarmalatorParamsVersions[id]++;
}

void appMain()
{
    DEBUG_PRINT("SWARMALATOR SNIFFER running\n");

    // Init the swarmalator params for all agents
    for (int i = 0; i < MAX_ADDRESS; i++) {
        Ks[i] = 0.0f;
        Js[i] = 0.0f;
        As[i] = 1.0f;
        Bs[i] = 1.0f;
        naturalFrequencies[i] = 0.0f;
        startingPhases[i] = 0.0f;
        swarmalatorParamsVersions[i] = 0;
    }

    initP2P();
    initPeerStates();

    broadcastTimer = xTimerCreate("SendCommandTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(broadcastTimer, 20);

    while (1) {
        vTaskDelay(M2T(SNIFFER_PRINT_PERIOD_MS));
        printPeers();
    }
}

#define DEFINE_SWARMALATOR_CB(AGENT_ID) \
    static void swarmalatorParamsCB_##AGENT_ID(void) { \
        swarmalatorParamsCB(AGENT_ID); \
    }

#define SWARMALATOR_AGENT_PARAMS(AGENT_ID) \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_K, &Ks[AGENT_ID], swarmalatorParamsCB_##AGENT_ID) \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_J, &Js[AGENT_ID], swarmalatorParamsCB_##AGENT_ID) \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_A, &As[AGENT_ID], swarmalatorParamsCB_##AGENT_ID) \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_B, &Bs[AGENT_ID], swarmalatorParamsCB_##AGENT_ID) \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_naturalFrequency, &naturalFrequencies[AGENT_ID], swarmalatorParamsCB_##AGENT_ID) \
    PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, AGENT_ID##_phase, &startingPhases[AGENT_ID], swarmalatorParamsCB_##AGENT_ID)

DEFINE_SWARMALATOR_CB(0)
DEFINE_SWARMALATOR_CB(1)
DEFINE_SWARMALATOR_CB(2)
DEFINE_SWARMALATOR_CB(3)
DEFINE_SWARMALATOR_CB(4)
DEFINE_SWARMALATOR_CB(5)
DEFINE_SWARMALATOR_CB(6)
DEFINE_SWARMALATOR_CB(7)
DEFINE_SWARMALATOR_CB(8)
DEFINE_SWARMALATOR_CB(9)

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