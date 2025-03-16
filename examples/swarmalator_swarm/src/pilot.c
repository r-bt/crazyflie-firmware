#include "choose_app.h"
#ifdef BUILD_PILOT_APP

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "common.h"
#include "configblock.h"
#include "movement.h"
#include "p2p_interface.h"
#include "param_log_interface.h"
#include "settings.h"
#include "supervisor.h"
#include "task.h"
#include "timers.h"

#define DEBUG_MODULE "SWARMALATOR_PILOT"
#include "debug.h"

static xTimerHandle sendPosTimer;
static xTimerHandle stateTransitionTimer;

static bool hasInit = false;

// Copter information

static uint8_t my_id;
enum State state = STATE_IDLE;

static uint32_t now_ms = 0;
static uint32_t position_lock_start_time_ms = 0;

// Timer functions

static void broadcastData(xTimerHandle timer)
{
    uint32_t nowMs = T2M(xTaskGetTickCount());

    copter_full_state_t fullState;

    fullState.id = my_id;
    fullState.state = state;
    fullState.battery_voltage = compressVoltage(getVoltage());
    fullState.timestamp = nowMs;
    fullState.position.x = getX();
    fullState.position.y = getY();
    fullState.position.z = getZ();

    broadcastToPeers(&fullState, nowMs);
}

static void stateTransition(xTimerHandle timer)
{
    DEBUG_PRINT("State is %u\n", state);

    if (supervisorIsTumbled()) {
        state = STATE_CRASHED;
    } else if (isBatteryLow() && (state == STATE_HOVERING || state == STATE_EXECUTING_SWARMALATOR)) {
        DEBUG_PRINT("Battery low, going home\n");
        // TODO: Instruct commander to go home
        state = STATE_GOING_HOME;
    }

    now_ms = T2M(xTaskGetTickCount());

    switch (state) {
    case STATE_IDLE:
        DEBUG_PRINT("State: IDLE; Waiting for position lock\n");
        resetPosLock();
        position_lock_start_time_ms = now_ms;
        state = STATE_WAIT_FOR_POSITION_LOCK;
        break;
    case STATE_WAIT_FOR_POSITION_LOCK:
        if (hasPosLock()) {
            DEBUG_PRINT("State: WAIT_FOR_POSITION_LOCK; Position lock acquired\n");
            state = STATE_WAIT_FOR_TAKE_OFF;
        }
        break;
    default:
        break;
    }
}

void appMain()
{

    if (hasInit) {
        return;
    }

    // Get Crazyflie ID (the last byte of the radio address)
    uint64_t address = configblockGetRadioAddress();
    my_id = (uint8_t)((address) & 0x00000000ff);

    DEBUG_PRINT("SWARMALATOR PILOT running on %u\n", my_id);

    // Init the parameter and logging interface
    initParamLogInterface();

    // Init P2P
    initP2P();
    initPeerStates();

    // Create timer to send position state to other Crazyflies
    sendPosTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(sendPosTimer, 20);

    // Create ticker to run the control loop
    stateTransitionTimer = xTimerCreate("StateTransitionTimer", M2T(CALC_NEXT_PERIOD_MS), pdTRUE, NULL, stateTransition);
    xTimerStart(stateTransitionTimer, 20);

    hasInit = true;
}

#endif // BUILD_PILOT_APP