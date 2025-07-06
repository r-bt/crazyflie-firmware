#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "param_log_interface.h"
#include "sensors.h"

#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.005f
#define CHARGED_FOR_TAKEOFF_VOLTAGE 3.5f

/**
 * Resets the position lock
 */
void resetPosLock();

/**
 * Checks if the Crazyflie has a position lock
 *
 * Compares the last LOCK_LENGTH velocity measurments to determine if the Crazyflie is stationary
 *
 * @return True if the Crazyflie has a position lock, false otherwise
 */
bool hasPosLock();

/**
 * Checks if the Crazyflie is charged for takeoff
 *
 * Checks if the battery voltage is above the takeoff threshold
 *
 * @return True if the Crazyflie is charged for takeoff, false otherwise
 */
bool chargedForTakeoff();

#endif // MOVEMENT_H