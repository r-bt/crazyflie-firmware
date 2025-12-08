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

void initSwarmalator(uint8_t my_id, plane_t* boundaryPlanes, int numBoundaryPlanes)
{
    swarmalator_params_t* params = getSwarmalatorParams();

    phase = params->startingPhase;

    s_boundaryPlanes = boundaryPlanes;
    s_numBoundaryPlanes = numBoundaryPlanes;
}

float getJ1Value(int my_id, float xPos, float yPos, float zPos) {

    swarmalator_params_t* params = getSwarmalatorParams();

    float distToTarget = sqrtf(powf(params->targetX - xPos, 2) + powf(params->targetY - yPos, 2) + powf(params->targetZ - zPos, 2));
    float minDist = distToTarget;
    float maxDist = distToTarget;

    for (uint8_t i = 1; i < MAX_ADDRESS; i++) {
        if (i != my_id && peerLocalizationIsIDActive(i)) {
            peerLocalizationOtherPosition_t *peerPosition = peerLocalizationGetPositionByID(i);

            if (peerPosition == NULL) {
                continue;
            }

            float distance = sqrtf(powf(params->targetX - peerPosition->pos.x, 2) + powf(params->targetY - peerPosition->pos.y, 2) + powf(params->targetZ - peerPosition->pos.z, 2));

            if (distance < minDist) {
                minDist = distance;
            }
            if (distance > maxDist) {
                maxDist = distance;
            }
        }
    }

    float J_val = params->alpha * fabsf((distToTarget - minDist) / (maxDist - minDist));

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

    // float J = params->targetSet ? getJ1Value(my_id, x_pos, y_pos, z_pos) : params->J;
    float J = calculate_safe_J(my_id, params->targetSet ? getJ1Value(my_id, x_pos, y_pos, z_pos) : params->J);

    // Loop over all the other agents (note start at 1 since we don't include the tower)
    for (uint8_t i = 1; i < MAX_ADDRESS; i++) {
        if (i != my_id && peerLocalizationIsIDActive(i)) {
            numActiveCopter++;

            peerLocalizationOtherPosition_t *peerPosition = peerLocalizationGetPositionByID(i);
            copter_full_state_t peer = getPeerState(i);
            
            if (peerPosition == NULL) {
                continue;
            }

            float thetaDiff = peer.phase - phase;

            #ifdef THREE_D_MODE
                float distance = sqrtf(powf(peerPosition->pos.x - x_pos, 2) + powf(peerPosition->pos.y - y_pos, 2) + powf(peerPosition->pos.z - z_pos, 2));
            #else
                float distance = sqrtf(powf(peerPosition->pos.x - x_pos, 2) + powf(peerPosition->pos.y - y_pos, 2));
            #endif

            if (distance == 0.0f) {
                distance = 0.01f; // Avoid division by zero
            }

            phase_sum += (sinf(thetaDiff) / distance);
            v_x_sum += ((peerPosition->pos.x - x_pos) / distance) * (params->A + J * cosf(thetaDiff)) - (params->B * (peerPosition->pos.x - x_pos) / (distance * distance));
            v_y_sum += ((peerPosition->pos.y - y_pos) / distance) * (params->A + J * cosf(thetaDiff)) - (params->B * (peerPosition->pos.y - y_pos) / (distance * distance));

            #ifdef THREE_D_MODE
                v_z_sum += ((peerPosition->pos.z - z_pos) / distance) * (params->A + J * cosf(thetaDiff)) - (params->B * (peerPosition->pos.z - z_pos) / (distance * distance));
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

        if (!plane.doesRepulse) {
            continue;
        }

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

        plane_v_x_sum += ((plane.normal.x / (distance * distance)) * params->B);
        plane_v_y_sum += ((plane.normal.y / (distance * distance)) * params->B);

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

float calculate_safe_J(int my_id, float J_target) {
    swarmalator_params_t* params = getSwarmalatorParams();

    float myX = getX();
    float myY = getY();
#ifdef THREE_D_MODE
    float myZ = getZ();
#endif
    float myPhase = getPhase();

    float centroid_x = 0.0f;
    float centroid_y = 0.0f;
#ifdef THREE_D_MODE
    float centroid_z = 0.0f;
#endif
    float Lfb0[3] = {0,0,0};
    float Lgb0[3] = {0,0,0};

    uint8_t numActiveCopter = 0;

    for (uint8_t i = 1; i < MAX_ADDRESS; i++) {
        if (i == my_id || !peerLocalizationIsIDActive(i)) continue;

        peerLocalizationOtherPosition_t *peerPos = peerLocalizationGetPositionByID(i);
        if (!peerPos) continue;

        copter_full_state_t peer = getPeerState(i);

        float dx = peerPos->pos.x - myX;
        float dy = peerPos->pos.y - myY;
#ifdef THREE_D_MODE
        float dz = peerPos->pos.z - myZ;
#endif

        float distance2 = dx*dx + dy*dy
#ifdef THREE_D_MODE
                          + dz*dz
#endif
                          ;
        float distance = sqrtf(distance2);
        if (distance < 1e-2f) distance = 1e-2f; // avoid div0

        float thetaDiff = peer.phase - myPhase;

        // accumulate centroid
        centroid_x += peerPos->pos.x;
        centroid_y += peerPos->pos.y;
#ifdef THREE_D_MODE
        centroid_z += peerPos->pos.z;
#endif

        // accumulate Lfb0
        float A = params->A;
        float B = params->B;
        float invDist = 1.0f / distance;
        float invDist2 = 1.0f / (distance*distance);

        Lfb0[0] += A*dx*invDist - B*dx*invDist2;
        Lfb0[1] += A*dy*invDist - B*dy*invDist2;
#ifdef THREE_D_MODE
        Lfb0[2] += A*dz*invDist - B*dz*invDist2;
#endif

        // accumulate Lgb0
        float cosTheta = cosf(thetaDiff);
        Lgb0[0] += dx*invDist*cosTheta;
        Lgb0[1] += dy*invDist*cosTheta;
#ifdef THREE_D_MODE
        Lgb0[2] += dz*invDist*cosTheta;
#endif

        numActiveCopter++;
    }

    if (numActiveCopter == 0) return J_target; // no neighbors

    // Average centroid and Lfb/Lgb
    centroid_x /= numActiveCopter;
    centroid_y /= numActiveCopter;
#ifdef THREE_D_MODE
    centroid_z /= numActiveCopter;
#endif

    Lfb0[0] /= numActiveCopter;
    Lfb0[1] /= numActiveCopter;
#ifdef THREE_D_MODE
    Lfb0[2] /= numActiveCopter;
#endif

    Lgb0[0] /= numActiveCopter;
    Lgb0[1] /= numActiveCopter;
#ifdef THREE_D_MODE
    Lgb0[2] /= numActiveCopter;
#endif

    // Compute center difference
#ifdef THREE_D_MODE
    float center_diff[3] = { myX - centroid_x, myY - centroid_y, myZ - centroid_z };
#else
    float center_diff[3] = { myX - centroid_x, myY - centroid_y, 0.0f };
#endif

    float norm2 = center_diff[0]*center_diff[0] + center_diff[1]*center_diff[1]
#ifdef THREE_D_MODE
                  + center_diff[2]*center_diff[2]
#endif
                  ;

    float rMin = 0.4f;
    float rMax = 0.6f;

    float b1 = norm2 - rMin*rMin;
    float b2 = rMax*rMax - norm2;

    float Lfb1 = 2*(center_diff[0]*Lfb0[0] + center_diff[1]*Lfb0[1]
#ifdef THREE_D_MODE
                     + center_diff[2]*Lfb0[2]
#endif
                     );
    float Lgb1 = 2*(center_diff[0]*Lgb0[0] + center_diff[1]*Lgb0[1]
#ifdef THREE_D_MODE
                     + center_diff[2]*Lgb0[2]
#endif
                     );

    float Lfb2 = -Lfb1;
    float Lgb2 = -Lgb1;

    float u_min = -1e30f;
    float u_max = 1e30f;
    float p = 10.0f;

    // Constraint 1
    if (fabsf(Lgb1) > 1e-9f) {
        float rhs1 = -(Lfb1 + p*b1);
        if (Lgb1 > 0) {
            if (rhs1/Lgb1 > u_min) u_min = rhs1/Lgb1;
        } else {
            if (rhs1/Lgb1 < u_max) u_max = rhs1/Lgb1;
        }
    }

    // Constraint 2
    if (fabsf(Lgb2) > 1e-9f) {
        float rhs2 = -(Lfb2 + p*b2);
        if (Lgb2 > 0) {
            if (rhs2/Lgb2 > u_min) u_min = rhs2/Lgb2;
        } else {
            if (rhs2/Lgb2 < u_max) u_max = rhs2/Lgb2;
        }
    }

    float J_safe = fminf(fmaxf(J_target, u_min), u_max);
    return J_safe;
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