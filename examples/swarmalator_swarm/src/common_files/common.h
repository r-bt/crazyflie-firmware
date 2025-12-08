/**
 * @file common.h
 *
 * @brief Common header file for the decentralized swarm
 *
 * This file contains the common header files and definitions used by the decentralized swarm.
 *
 * @author Richard Beattie
 * @date 10/03/2025
 */

#include <stdint.h>

#pragma once

enum State {
    STATE_IDLE = 0,
    STATE_WAIT_FOR_POSITION_LOCK = 1,

    STATE_WAIT_FOR_TAKE_OFF = 2,
    STATE_QUEUED_FOR_TAKE_OFF = 3,
    STATE_PREPARING_FOR_TAKE_OFF = 4,
    STATE_TAKING_OFF = 5,
    STATE_HOVERING = 6,
    STATE_EXECUTING_SWARMALATOR = 7,
    STATE_GOING_HOME = 8,
    STATE_PREPARING_TO_LAND = 9,
    STATE_WAITING_AT_PAD = 10,
    STATE_LANDING = 11,
    STATE_CRASHED = 16,
    STATE_SNIFFING = 17,
    STATE_UNKNOWN = 255,
};

uint8_t getMyId(void);

typedef struct {
    float K;
    float J;
    float A;
    float B;
    float naturalFrequency;
    float startingPhase;
    uint8_t targetSet;
    float targetX;
    float targetY;
    float targetZ;
    float alpha;
} swarmalator_params_t;