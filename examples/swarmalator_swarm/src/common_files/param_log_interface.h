/**
 * @file param_log_interface.h
 *
 * @brief Interface for getting and setting parameters and logging data
 *
 * @author Richard Beattie
 * @date 09/03/2025
 */

#pragma once

#include "log.h"
#include "param.h"
#include "pm.h"

/**
 * Get the x position of the Crazyflie
 *
 * @return The x position of the Crazyflie
 */
float getX();

/**
 * Get the y position of the Crazyflie
 *
 * @return The y position of the Crazyflie
 */
float getY();

/**
 * Get the z position of the Crazyflie
 *
 * @return The z position of the Crazyflie
 */
float getZ();

/**
 * Get the x velocity of the Crazyflie
 *
 * @return The x velocity of the Crazyflie
 */
float getVX();

/**
 * Get the y velocity of the Crazyflie
 *
 * @return The y velocity of the Crazyflie
 */
float getVY();

/**
 * Get the z velocity of the Crazyflie
 *
 * @return The z velocity of the Crazyflie
 */
float getVZ();

/**
 * Get velocity in body frame x.
 *
 * @link https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#kalmanstatepx
 */
float getVarPX();

/**
 * Get velocity in body frame y.
 *
 * @link https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#kalmanstatepy
 */
float getVarPY();

/**
 * Get velocity in body frame z.
 *
 * @link https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#kalmanstatepz
 */
float getVarPZ();

/**
 * Get the voltage of the Crazyflie
 *
 * @return The voltage of the Crazyflie
 */
float getVoltage();

/**
 * Initialize the parameter and logging interface
 */
void initParamLogInterface();

/**
 * Check if Crazyflie has low battery
 *
 * @return True if battery is low, false otherwise
 */
bool isBatteryLow(void);

/**
 * Check if Crazyflie can see Lighthouse base stations
 */
bool isLighthouseAvailable(void);