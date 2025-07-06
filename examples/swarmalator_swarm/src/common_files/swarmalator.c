#include "swarmalator.h"
#include "p2p_interface.h"
#include "param_log_interface.h"
#include "settings.h"

static uint32_t prevUpdate_ms = 0;

/**
 * Swarmalator parameters
 */
float K = 1.0f; // Phase synchronization strength
float J = 1.0f; // Like-attracts-like strength
float A = 1.0f; // Attraction strength
float B = 2.0f; // Repulsion strength

float naturalFrequency = 1.0f;
float phase = 0.0f; // Phase of the copter (radians)

float desiredDeltaX = 0.0f; // Desired x position
float desiredDeltaY = 0.0f; // Desired y position
float duration = 0.0f; // Duration of the trajectory

int agents = 3;

void initSwarmalator(uint8_t my_id)
{
    int pos = (my_id - 1);

    // Linspace between 0 and 2*pi
    float phaseStep = (2.0f * (float)M_PI) / agents;
    phase = pos * phaseStep;
}

void update_swarmalator(uint8_t my_id)
{

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

    for (uint8_t i = 0; i < MAX_ADDRESS; i++) {
        if (i != my_id && peerLocalizationIsIDActive(i)) {
            numActiveCopter++;

            copter_full_state_t peer = getPeerState(i);

            float thetaDiff = peer.phase - phase;
            float distance = sqrtf(powf(peer.position.x - x_pos, 2) + powf(peer.position.y - y_pos, 2));

            if (distance == 0.0f) {
                distance = 0.01f; // Avoid division by zero
            }

            phase_sum += (sinf(thetaDiff) / distance);
            v_x_sum += ((peer.position.x - x_pos) / distance) * (A + J * cosf(thetaDiff)) - (B * (peer.position.x - x_pos) / (distance * distance));
            v_y_sum += ((peer.position.y - y_pos) / distance) * (A + J * cosf(thetaDiff)) - (B * (peer.position.y - y_pos) / (distance * distance));
        }
    }

    if (numActiveCopter > 0) {
        phase_sum *= K / numActiveCopter;
        v_x_sum /= numActiveCopter;
        v_y_sum /= numActiveCopter;
    }

    phase += (naturalFrequency + phase_sum) * ((T2M(xTaskGetTickCount()) - prevUpdate_ms) / 1000.0f);

    // Normalize the phase to [0, 2*pi]
    if (phase > 2.0f * (float)M_PI) {
        phase -= 2.0f * (float)M_PI;
    } else if (phase < 0.0f) {
        phase += 2.0f * (float)M_PI;
    }

    // Update the desired position based on the velocity
    desiredDeltaX = v_x_sum * ((T2M(xTaskGetTickCount()) - prevUpdate_ms) / 1000.0f);
    desiredDeltaY = v_y_sum * ((T2M(xTaskGetTickCount()) - prevUpdate_ms) / 1000.0f);
    duration = (T2M(xTaskGetTickCount()) - prevUpdate_ms) / 1000.0f;

    prevUpdate_ms = T2M(xTaskGetTickCount());
}

float getPhase()
{
    return phase;
}

float getDesiredDeltaX()
{
    return desiredDeltaX;
}

float getDesiredDeltaY()
{
    return desiredDeltaY;
}

float getDuration()
{
    return duration;
}