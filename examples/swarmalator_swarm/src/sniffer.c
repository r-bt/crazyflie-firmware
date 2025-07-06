#include "choose_app.h"
#ifdef BUILD_SNIFFER_APP

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define DEBUG_MODULE "P2P"

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

    // if (nowMs < endBroadcastTime) {
    copter_full_state_t fullState;

    fullState.id = 0; // The sniffer
    fullState.state = STATE_SNIFFING;
    fullState.battery_voltage = 0;
    fullState.timestamp = nowMs;
    fullState.position.x = 0.0f;
    fullState.position.y = 0.0f;
    fullState.position.z = 0.0f;

    broadcastToPeers(&fullState, nowMs);
    // }
}

static uint8_t isExperimentRunningVal = 0;
static void isExperimentRunningCB(void)
{
    setIsRunning(isExperimentRunningVal);

    endBroadcastTime = T2M(xTaskGetTickCount()) + 1000; // Broadcast for 1 second
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

PARAM_GROUP_START(app)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, isExperimentRunning, &isExperimentRunningVal, &isExperimentRunningCB)
PARAM_GROUP_STOP(app)

#endif // BUILD_SNIFFER_APP