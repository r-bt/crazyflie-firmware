#include "choose_app.h"
#ifdef BUILD_PILOT_APP

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "common.h"
#include "configblock.h"
#include "crtp_commander_high_level.h"
#include "led_ring.h"
#include "movement.h"
#include "p2p_interface.h"
#include "param_log_interface.h"
#include "settings.h"
#include "supervisor.h"
#include "swarmalator.h"
#include "task.h"
#include "timers.h"
#include "commander.h"

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

static struct HSV ledColor = { 0, 255, 255 }; // Initial color for the LED ring

static uint32_t random_time_for_next_event_ms = 0;

static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t stabilizeEndTime_ms;

static bool isCrashInitialized = false;
static bool isInFlightArea = true;

// Boundary planes to prevent agents from existing the flight area (i.e. the area that the lighthouse can track them in)
static plane_t boundaryPlanes[] = {
    { .point = {.x = 0, .y = 0, .z = 0}, .normal = {.x = 0, .y = 0, .z = 1}, .doesRepulse = true },
    { .point = {.x = 0, .y = 0, .z = 1.9}, .normal = {.x = 0, .y = 0, .z = -1}, .doesRepulse = true },
    { .point = {.x = 1.7, .y = 0, .z = 0}, .normal = {.x = -1, .y = 0, .z = 0}, .doesRepulse = true },
    { .point = {.x = -1.7, .y = 0, .z = 0}, .normal = {.x = 1, .y = 0, .z = 0}, .doesRepulse = true },
    { .point = {.x = 0, .y = 2.1, .z = 0}, .normal = {.x = 0, .y = -1, .z = 0}, .doesRepulse = true },
    { .point = {.x = 0, .y = -2.1, .z = 0}, .normal = {.x = 0, .y = 1, .z = 0}, .doesRepulse = true }
};
int numBoundaryPlanes = sizeof(boundaryPlanes) / sizeof(boundaryPlanes[0]);

// Timer functions

uint32_t get_next_random_timeout(uint32_t now_ms)
{
    uint32_t extra = (rand() % (TAKE_OFF_TIME_MAX - TAKE_OFF_TIME_MIN)) + TAKE_OFF_TIME_MIN;
    uint32_t timeout = now_ms + extra;
    DEBUG_PRINT("Next random timeout dt: %lu \n", extra);
    return timeout;
}

#ifdef THREE_D_MODE
    static void setVelocitySetpoint3D(setpoint_t *setpoint, float vx, float vy, float vz, float yawrate)
    {
        setpoint->mode.z = modeVelocity;
        setpoint->velocity.z = vz;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = yawrate;
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->velocity.x = vx;
        setpoint->velocity.y = vy;

        setpoint->velocity_body = false;
    }
#else
    static void setVelocitySetpoint2D(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
    {
        setpoint->mode.z = modeAbs;
        setpoint->position.z = z;
        setpoint->mode.yaw = modeVelocity;
        setpoint->attitudeRate.yaw = yawrate;
        setpoint->mode.x = modeVelocity;
        setpoint->mode.y = modeVelocity;
        setpoint->velocity.x = vx;
        setpoint->velocity.y = vy;

        setpoint->velocity_body = false;
    }
#endif

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
    fullState.phase = getPhase();
    fullState.swarmalatorParamsVersion = getSwarmalatorsParamsVersion();

    broadcastToPeers(&fullState, nowMs);
}

static void startTakeOffSequence()
{
    Position pad_sampler = { 0.0f, 0.0f, 0.0f };

    for (uint8_t i = 0; i < NUMBER_OF_PAD_SAMPLES; i++) {
        pad_sampler.x += getX();
        pad_sampler.y += getY();
        pad_sampler.z += getZ();
        vTaskDelay(M2T(10));
    }

    padX = pad_sampler.x / NUMBER_OF_PAD_SAMPLES;
    padY = pad_sampler.y / NUMBER_OF_PAD_SAMPLES;
    padZ = pad_sampler.z / NUMBER_OF_PAD_SAMPLES;
    DEBUG_PRINT("Base position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ);

    DEBUG_PRINT("Taking off...\n");
    crtpCommanderHighLevelTakeoff(TAKE_OFF_HEIGHT, 1.0);
}

static void stateTransition(xTimerHandle timer)
{
    setpoint_t setpoint;

    if (supervisorIsTumbled()) {
        state = STATE_CRASHED;
    } else if (isBatteryLow() && (state == STATE_HOVERING || state == STATE_EXECUTING_SWARMALATOR)) {
        DEBUG_PRINT("Battery low, going home\n");
        // TODO: Instruct commander to go home
        state = STATE_PREPARING_TO_LAND;
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
        } else {
            DEBUG_PRINT("State: WAIT_FOR_POSITION_LOCK; Waiting for position lock\n");
        }
        break;
    case STATE_WAIT_FOR_TAKE_OFF:
        if (!chargedForTakeoff()) {
            DEBUG_PRINT("Battery not charged for take off\n");
            // do nothing, wait for the battery to be charged
        } else if (isExperimentRunning() && isInFlightArea) {
            DEBUG_PRINT("Entering takeoff queue...\n");
            state = STATE_QUEUED_FOR_TAKE_OFF;
            initSwarmalator(my_id, boundaryPlanes, numBoundaryPlanes); // reset swarmalator
        }
        break;
    case STATE_QUEUED_FOR_TAKE_OFF:
        if (!chargedForTakeoff()) {
            state = STATE_WAIT_FOR_TAKE_OFF;
        } else {
            DEBUG_PRINT("Preparing for take off...\n");
            if (supervisorRequestArming(true)) {
                random_time_for_next_event_ms = get_next_random_timeout(now_ms);
                state = STATE_PREPARING_FOR_TAKE_OFF;
            }
        }
        break;
    case STATE_PREPARING_FOR_TAKE_OFF:
        if (now_ms > random_time_for_next_event_ms) {
            DEBUG_PRINT("Taking off...\n");
            startTakeOffSequence();
            state = STATE_TAKING_OFF;
        }
        break;
    case STATE_TAKING_OFF:
        if (crtpCommanderHighLevelIsTrajectoryFinished()) {
            DEBUG_PRINT("Hovering, waiting for command to start\n");
            state = STATE_HOVERING;
        }
        break;
    case STATE_HOVERING:
        if (!isExperimentRunning() || !isInFlightArea) {
            DEBUG_PRINT("Not running, going home\n");
            random_time_for_next_event_ms = get_next_random_timeout(now_ms);
            stabilizeEndTime_ms = now_ms + STABILIZE_TIMEOUT;
            state = STATE_PREPARING_TO_LAND;

            commanderRelaxPriority();
            crtpCommanderHighLevelGoTo2(getX(), getY(), getZ(), 0.0f, 1.0f, false, true); // stay in place
        } else {
            update_swarmalator(my_id);

            // Update the phase
            ledColor.hue = radians_to_hue(getPhase());
            setRingSolidColor(hsvToRgb(ledColor));

            // Go to the new position
            // DEBUG_PRINT("Going to (%f, %f) in %f seconds\n", (double)getDesiredVx(), (double)getDesiredVy(), (double)getDuration());
            // crtpCommanderHighLevelGoTo2(getDesiredDeltaX(), getDesiredDeltaY(), 0, 0.0f, 0.1f, true, false);
            #ifdef THREE_D_MODE
                setVelocitySetpoint3D(&setpoint, getDesiredVx(), getDesiredVy(), getDesiredVz(), 0);
            #else
                setVelocitySetpoint2D(&setpoint, getDesiredVx(), getDesiredVy(), TAKE_OFF_HEIGHT, 0);
            #endif
            commanderSetSetpoint(&setpoint, 3);

            float x_pos = getX();
            float y_pos = getY();
            float z_pos = getZ();

            for (int i = 0; i < numBoundaryPlanes; i++) {
                plane_t plane = boundaryPlanes[i];

                float distance = ((plane.normal.x * (x_pos - plane.point.x)) +
                                  (plane.normal.y * (y_pos - plane.point.y)) +
                                  (plane.normal.z * (z_pos - plane.point.z)));

                if (distance < -0.05f) {
                    DEBUG_PRINT("Warning: approaching boundary plane %d, distance: %f\n", i, (double)distance);
                    isInFlightArea = false;
                    break;
                }
            }
        }
        break;
    case STATE_PREPARING_TO_LAND:
        // Turn off the LED ring
        if (crtpCommanderHighLevelIsTrajectoryFinished()) {
            setRingSolidColorRGB(0, 0, 0);
            crtpCommanderHighLevelLand(padZ + LANDING_HEIGHT, GO_TO_PAD_DURATION);

            state = STATE_GOING_HOME;
        }
        break;
    case STATE_GOING_HOME:
        if (crtpCommanderHighLevelIsTrajectoryFinished()) {
            DEBUG_PRINT("Over pad, stabilizing position\n");
            stabilizeEndTime_ms = now_ms + STABILIZE_TIMEOUT;
            state = STATE_WAITING_AT_PAD;
        }
        break;
    case STATE_WAITING_AT_PAD:
        if (now_ms > stabilizeEndTime_ms || (fabs((padZ + LANDING_HEIGHT) - getZ()) < MAX_PAD_ERR)) {
            if (now_ms > stabilizeEndTime_ms) {
                DEBUG_PRINT("Warning: timeout!\n");
            }

            DEBUG_PRINT("Landing...\n");
            crtpCommanderHighLevelLand(padZ, LANDING_DURATION);
            state = STATE_LANDING;
        }
        break;
    case STATE_LANDING:
        if (crtpCommanderHighLevelIsTrajectoryFinished()) {
            DEBUG_PRINT("Landed\n");
            crtpCommanderHighLevelStop();
            if (supervisorRequestArming(false)) {
                initSwarmalator(my_id, boundaryPlanes, numBoundaryPlanes); // reset swarmalator
                state = STATE_WAIT_FOR_TAKE_OFF;
            }
        }
        break;
    case STATE_CRASHED:
        // Disbale the LED ring
        setRingSolidColorRGB(0, 0, 0);
        if (!isCrashInitialized) {
            crtpCommanderHighLevelStop();
            DEBUG_PRINT("Crashed, running crash sequence\n");
            isCrashInitialized = true;
            // maybe need to disarm here
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

    my_id = getMyId();

    DEBUG_PRINT("SWARMALATOR PILOT running on %u\n", my_id);

    // Init the parameter and logging interface
    initParamLogInterface();

    // Init P2P
    initP2P();
    initPeerStates();

    // Init the LED ring
    initLedRing();

    // Create timer to send position state to other Crazyflies
    sendPosTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(sendPosTimer, 20);

    // Create ticker to run the control loop
    stateTransitionTimer = xTimerCreate("StateTransitionTimer", M2T(CALC_NEXT_PERIOD_MS), pdTRUE, NULL, stateTransition);
    xTimerStart(stateTransitionTimer, 20);

    hasInit = true;
}

#endif // BUILD_PILOT_APP