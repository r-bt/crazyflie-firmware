#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "param_log_interface.h"
#include "sensors.h"

#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f

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

#endif // MOVEMENT_H