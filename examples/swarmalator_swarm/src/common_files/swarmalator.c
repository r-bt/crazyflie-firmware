#include "swarmalator.h"
#include "p2p_interface.h"
#include "param_log_interface.h"
#include "settings.h"
#include "debug.h"

#define DEBUG_MODULE "SWARMALATOR"

static uint32_t prevUpdate_ms = 0;

float phase = 0.0f; // Phase of the copter (radians)

float desiredVx = 0.0f; // Desired x velocity
float desiredVy = 0.0f; // Desired y velocity

#ifdef THREE_D_MODE
float desiredVz = 0.0f; // Desired z velocity
#endif

float duration = 0.0f; // Duration of the trajectory

static plane_t *s_boundaryPlanes;
static int s_numBoundaryPlanes = 0;

bool isTargetSet = false;
float target[3] = { 1.0f, 1.0f, 0.75f};

float alpha = 1.0f; // Weighting factor for J1

void initSwarmalator(uint8_t my_id, plane_t* boundaryPlanes, int numBoundaryPlanes)
{
    swarmalator_params_t* params = getSwarmalatorParams();

    phase = params->startingPhase;

    s_boundaryPlanes = boundaryPlanes;
    s_numBoundaryPlanes = numBoundaryPlanes;
}

float getJ1Value(int my_id, float xPos, float yPos, float zPos) {

    float distToTarget = sqrtf(powf(target[0] - xPos, 2) + powf(target[1] - yPos, 2) + powf(target[2] - zPos, 2));
    float minDist = distToTarget;
    float maxDist = distToTarget;

    for (uint8_t i = 1; i < MAX_ADDRESS; i++) {
        if (i != my_id && peerLocalizationIsIDActive(i)) {
            copter_full_state_t peer = getPeerState(i);

            float distance = sqrtf(powf(target[0] - peer.position.x, 2) + powf(target[1] - peer.position.y, 2) + powf(target[2] - peer.position.z, 2));

            if (distance < minDist) {
                minDist = distance;
            }
            if (distance > maxDist) {
                maxDist = distance;
            }
        }
    }

    float J_val = alpha * fabsf((distToTarget - minDist) / (maxDist - minDist));

    DEBUG_PRINT("J1 Value: %f, Dist to Target: %f, Min Dist: %f, Max Dist: %f\n", (double)J_val, (double)distToTarget, (double)minDist, (double)maxDist);

    return J_val;
}

void update_swarmalator(uint8_t my_id)
{
    swarmalator_params_t* params = getSwarmalatorParams();

    if (prevUpdate_ms == 0) {
        prevUpdate_ms = T2M(xTaskGetTickCount());
    }

    // Update the angular velocity according to:
    // dtheta/dt = omega + K/N * SUM(sin(theta_j - theta_i) / abs(x_j - x_i))

    // Update the velocity according to:
    // v_i = 1/N + SUM[(x_j - x_i) / abs(x_j - x_i) * (A + J * cos(theta_j - theta_i)) - B * (x_j - x_i) / abs(x_j - x_i)^2]
    uint8_t numActiveCopter = 0;

    float x_pos = getX();
    float y_pos = getY();

    float phase_sum = 0.0f;
    float v_x_sum = 0.0f;
    float v_y_sum = 0.0f;

    #ifdef THREE_D_MODE
        float z_pos = getZ();
        float v_z_sum = 0.0f;
    #endif

    float J = isTargetSet ? getJ1Value(my_id, x_pos, y_pos, z_pos) : params->J;

    // Loop over all the other agents (note start at 1 since we don't include the tower)
    for (uint8_t i = 1; i < MAX_ADDRESS; i++) {
        if (i != my_id && peerLocalizationIsIDActive(i)) {
            numActiveCopter++;

            copter_full_state_t peer = getPeerState(i);

            float thetaDiff = peer.phase - phase;

            #ifdef THREE_D_MODE
                float distance = sqrtf(powf(peer.position.x - x_pos, 2) + powf(peer.position.y - y_pos, 2) + powf(peer.position.z - z_pos, 2));
            #else
                float distance = sqrtf(powf(peer.position.x - x_pos, 2) + powf(peer.position.y - y_pos, 2));
            #endif

            if (distance == 0.0f) {
                distance = 0.01f; // Avoid division by zero
            }

            phase_sum += (sinf(thetaDiff) / distance);
            v_x_sum += ((peer.position.x - x_pos) / distance) * (params->A + J * cosf(thetaDiff)) - (params->B * (peer.position.x - x_pos) / (distance * distance));
            v_y_sum += ((peer.position.y - y_pos) / distance) * (params->A + J * cosf(thetaDiff)) - (params->B * (peer.position.y - y_pos) / (distance * distance));

            #ifdef THREE_D_MODE
                v_z_sum += ((peer.position.z - z_pos) / distance) * (params->A + J * cosf(thetaDiff)) - (params->B * (peer.position.z - z_pos) / (distance * distance));
            #endif
        }
    }

    if (numActiveCopter > 0) {
        phase_sum *= params->K / numActiveCopter;
        v_x_sum /= numActiveCopter;
        v_y_sum /= numActiveCopter;

        #ifdef THREE_D_MODE
            v_z_sum /= numActiveCopter;
        #endif
    }

    // Add repulsion from bounding planes
    float plane_v_x_sum = 0.0f;
    float plane_v_y_sum = 0.0f;

    #ifdef THREE_D_MODE
        float plane_v_z_sum = 0.0f;
    #endif

    for (int i = 0; i < s_numBoundaryPlanes; i++) {

        plane_t plane = s_boundaryPlanes[i];

        // Compute the distance from agent to the plane
        #ifdef THREE_D_MODE
            float distance = (x_pos - plane.point.x) * plane.normal.x + (y_pos - plane.point.y) * plane.normal.y + (z_pos - plane.point.z) * plane.normal.z;
        #else
            float distance = (x_pos - plane.point.x) * plane.normal.x + (y_pos - plane.point.y) * plane.normal.y;
        #endif

        if (distance == 0.0f) {
            distance = 0.01f; // Avoid division by zero
        }

        distance += (1 - PLANE_INFLECTION_DISTANCE); // Shift the distance so that repulsion becomes exponential at the inflection distance

        plane_v_x_sum += ((plane.normal.x / distance * distance) * params->B);
        plane_v_y_sum += ((plane.normal.y / distance * distance) * params->B);

        #ifdef THREE_D_MODE
            plane_v_z_sum += ((plane.normal.z / (distance * distance)) * params->B);
        #endif
    }

    v_x_sum += plane_v_x_sum / s_numBoundaryPlanes;
    v_y_sum += plane_v_y_sum / s_numBoundaryPlanes;
    #ifdef THREE_D_MODE
        v_z_sum += plane_v_z_sum / s_numBoundaryPlanes;
    #endif

    phase += (params->naturalFrequency + phase_sum) * ((T2M(xTaskGetTickCount()) - prevUpdate_ms) / 1000.0f);

    // Normalize the phase to [0, 2*pi]
    if (phase > 2.0f * (float)M_PI) {
        phase -= 2.0f * (float)M_PI;
    } else if (phase < 0.0f) {
        phase += 2.0f * (float)M_PI;
    }

    // Update the desired position based on the velocity
    desiredVx = v_x_sum;
    desiredVy = v_y_sum;

    #ifdef THREE_D_MODE
        desiredVz = v_z_sum;
    #endif

    duration = (T2M(xTaskGetTickCount()) - prevUpdate_ms) / 1000.0f;

    prevUpdate_ms = T2M(xTaskGetTickCount());
}

float getPhase()
{
    return phase;
}

float getDesiredVx()
{
    return desiredVx;
}

float getDesiredVy()
{
    return desiredVy;
}

#ifdef THREE_D_MODE
float getDesiredVz()
{
    return desiredVz;
}
#endif

float getDuration()
{
    return duration;
}