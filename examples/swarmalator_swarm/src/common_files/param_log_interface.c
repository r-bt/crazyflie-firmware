#include "param_log_interface.h"

static logVarId_t logIdVBat;
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdStateEstimateVx;
static logVarId_t logIdStateEstimateVy;
static logVarId_t logIdStateEstimateVz;
static logVarId_t logIdPmState;
static logVarId_t logIdKalmanVarPX;
static logVarId_t logIdKalmanVarPY;
static logVarId_t logIdKalmanVarPZ;
static logVarId_t logIdlighthouseEstBs0Rt;
static logVarId_t logIdlighthouseEstBs1Rt;

float getX()
{
    return logGetFloat(logIdStateEstimateX);
}

float getY()
{
    return logGetFloat(logIdStateEstimateY);
}

float getZ()
{
    return logGetFloat(logIdStateEstimateZ);
}

float getVX()
{
    return logGetFloat(logIdStateEstimateVx);
}

float getVY()
{
    return logGetFloat(logIdStateEstimateVy);
}

float getVZ()
{
    return logGetFloat(logIdStateEstimateVz);
}

float getVoltage()
{
    return logGetFloat(logIdVBat);
}

bool isBatteryLow()
{
    return logGetInt(logIdPmState) == lowPower;
}

float getVarPX()
{
    return logGetFloat(logIdKalmanVarPX);
}

float getVarPY()
{
    return logGetFloat(logIdKalmanVarPY);
}

float getVarPZ()
{
    return logGetFloat(logIdKalmanVarPZ);
}

bool isLighthouseAvailable(void)
{
    return logGetFloat(logIdlighthouseEstBs0Rt) >= 0.0f || logGetFloat(logIdlighthouseEstBs1Rt) >= 0.0f;
}

void initParamLogInterface()
{
    logIdVBat = logGetVarId("pm", "vbat");
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdStateEstimateVx = logGetVarId("stateEstimate", "vx");
    logIdStateEstimateVy = logGetVarId("stateEstimate", "vy");
    logIdStateEstimateVz = logGetVarId("stateEstimate", "vz");
    logIdPmState = logGetVarId("pm", "state");
    logIdKalmanVarPX = logGetVarId("kalman", "varPX");
    logIdKalmanVarPY = logGetVarId("kalman", "varPY");
    logIdKalmanVarPZ = logGetVarId("kalman", "varPZ");
    logIdlighthouseEstBs0Rt = logGetVarId("lighthouse", "estBs0Rt");
    logIdlighthouseEstBs1Rt = logGetVarId("lighthouse", "estBs1Rt");
}