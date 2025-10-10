/**
 * @file led_ring.h
 *
 * @brief Interface for interacting with the LED ring on the Crazyflie
 *
 * @author Richard Beattie
 * @date 17/03/2025
 */

#pragma once

#include "param.h"

struct RGB {
    uint8_t red; /** Red component (0-255) */
    uint8_t green; /** Green component (0-255) */
    uint8_t blue; /** Blue component (0-255) */
};

struct HSV {
    uint8_t hue; /** Hue component (0-255) */
    uint8_t saturation; /** Saturation component (0-255) */
    uint8_t value; /** Value component (0-255) */
};

/**
 * Convert radians to hue
 *
 * @param radians The angle in radians
 * @return The corresponding hue value (0-255)
 */
uint8_t radians_to_hue(float radians);

/**
 * Convert an HSV color to an RGB color
 *
 * @param hsv The HSV color to convert
 * @return The corresponding RGB color
 */
struct RGB hsvToRgb(struct HSV hsv);

/**
 * Initialize the LED ring parameters
 */
void initLedRing();

/**
 * Set the solid color of the LED ring
 *
 * @param color The RGB color to set the LED ring to
 *              (0-255 for each component)
 */
void setRingSolidColor(struct RGB color);

/**
 * Set the solid color of the LED ring using RGB values
 *
 * @param red The red component (0-255)
 * @param green The green component (0-255)
 * @param blue The blue component (0-255)
 */
void setRingSolidColorRGB(uint8_t red, uint8_t green, uint8_t blue);