/**
 * @file swarmalator.h
 *
 * @brief Implementation of the Swarmalator algorithm
 *
 * @author Richard Beattie
 * @date 06/04/2025
 */

#pragma once

#include "p2p_interface.h"

typedef struct {
    float K;
    float J;
    float A;
    float B;
    float naturalFrequency;
    float startingPhase;
} swarmalator_params_t;

/**
 * @brief Initializes the swarmalator algorithm
 */
void initSwarmalator(uint8_t my_id);

/**
 * @brief Updates the swarmalator algorithm based on the current state of peer copters
 */
void update_swarmalator(uint8_t my_id);

/**
 * @brief Gets the desired delta x of the Crazyflie
 *
 * @return The desired delta x of the Crazyflie
 */
float getDesiredVx();

/**
 * @brief Gets the desired delta y of the Crazyflie
 *
 * @return The desired delta y of the Crazyflie
 */
float getDesiredVy();

/**
 * @brief Gets the desired duration of the trajectory
 *
 * @return The desired duration of the trajectory
 */
float getDuration();

/**
 * @brief Gets the current phase of the Crazyflie (in radians)
 *
 * @return The current phase of the Crazyflie (in radians)
 */
float getPhase();
