/**
 * @file movement.c
 *
 * @brief Movement functions for the decentralized swarm
 *
 * @author Richard Beattie
 * @date 10/03/2025
 */

#include "movement.h"

static float posLockData[LOCK_LENGTH][3];
static uint32_t posLockWriteIndex;

void resetPosLock()
{
    posLockWriteIndex = 0;
    for (uint32_t i = 0; i < LOCK_LENGTH; i++) {
        posLockData[i][0] = FLT_MAX;
        posLockData[i][1] = FLT_MAX;
        posLockData[i][2] = FLT_MAX;
    }
}

bool hasPosLock()
{
    bool result = false;

    // Store the current state
    posLockData[posLockWriteIndex][0] = getVarPX();
    posLockData[posLockWriteIndex][1] = getVarPY();
    posLockData[posLockWriteIndex][2] = getVarPZ();

    posLockWriteIndex++;
    if (posLockWriteIndex >= LOCK_LENGTH) {
        posLockWriteIndex = 0;
    }

    // Check if we have a lock
    int count = 0;

    float lxMax = FLT_MIN;
    float lxMin = FLT_MAX;
    float lyMax = FLT_MIN;
    float lyMin = FLT_MAX;
    float lzMax = FLT_MIN;
    float lzMin = FLT_MAX;

    for (int i = 0; i < LOCK_LENGTH; i++) {
        if (posLockData[i][0] != FLT_MAX) {
            count++;

            lxMax = fmaxf(lxMax, posLockData[i][0]);
            lxMin = fminf(lxMin, posLockData[i][0]);
            lyMax = fmaxf(lyMax, posLockData[i][1]);
            lyMin = fminf(lyMin, posLockData[i][1]);
            lzMax = fmaxf(lzMax, posLockData[i][2]);
            lzMin = fminf(lzMin, posLockData[i][2]);
        }
    }

    result = (count >= LOCK_LENGTH) && (lxMax - lxMin < LOCK_THRESHOLD) && (lyMax - lyMin < LOCK_THRESHOLD) && (lzMax - lzMin < LOCK_THRESHOLD) && isLighthouseAvailable() && sensorsAreCalibrated();

    return result;
}