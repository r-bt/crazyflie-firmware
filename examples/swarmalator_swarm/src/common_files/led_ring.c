#include "led_ring.h"
#include "math.h"

static paramVarId_t paramLedEffect;
static paramVarId_t paramSolidRed;
static paramVarId_t paramSolidBlue;
static paramVarId_t paramSolidGreen;

uint8_t radians_to_hue(float radians)
{
    float hue = fmodf((radians / (2.0f * (float)M_PI)) * 255.0f, 255.0f);
    return (uint8_t)hue;
}


struct RGB hsvToRgb(struct HSV hsv)
{
    struct RGB rgb;
    uint8_t region, remainder, p, q, t;

    if (hsv.saturation == 0) {
        rgb.red = rgb.green = rgb.blue = hsv.value; // Achromatic (grey)
        return rgb;
    }

    region = hsv.hue / 43; // 43 is 255/6
    remainder = (hsv.hue - (region * 43)) * 6; // remainder is in [0, 255]

    p = (hsv.value * (255 - hsv.saturation)) >> 8;
    q = (hsv.value * (255 - ((hsv.saturation * remainder) >> 8))) >> 8;
    t = (hsv.value * (255 - ((hsv.saturation * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
    case 0:
        rgb.red = hsv.value;
        rgb.green = t;
        rgb.blue = p;
        break;
    case 1:
        rgb.red = q;
        rgb.green = hsv.value;
        rgb.blue = p;
        break;
    case 2:
        rgb.red = p;
        rgb.green = hsv.value;
        rgb.blue = t;
        break;
    case 3:
        rgb.red = p;
        rgb.green = q;
        rgb.blue = hsv.value;
        break;
    case 4:
        rgb.red = t;
        rgb.green = p;
        rgb.blue = hsv.value;
        break;
    default:
        rgb.red = hsv.value;
        rgb.green = p;
        rgb.blue = q;
        break;
    }

    return rgb;
}

void initLedRing()
{
    paramLedEffect = paramGetVarId("ring", "effect");
    paramSolidRed = paramGetVarId("ring", "solidRed");
    paramSolidBlue = paramGetVarId("ring", "solidBlue");
    paramSolidGreen = paramGetVarId("ring", "solidGreen");

    // Use the solid color effect
    paramSetInt(paramLedEffect, 7);
    paramSetInt(paramSolidRed, 0);
    paramSetInt(paramSolidBlue, 0);
    paramSetInt(paramSolidGreen, 0);
}

void setRingSolidColor(struct RGB color)
{
    setRingSolidColorRGB(color.red, color.green, color.blue);
}

void setRingSolidColorRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    paramSetInt(paramSolidRed, red);
    paramSetInt(paramSolidGreen, green);
    paramSetInt(paramSolidBlue, blue);
}