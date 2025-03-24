#include "Encoder.h"

Encoder::Encoder(gpio_num_t pinA, gpio_num_t pinB)
    : pinA(pinA), pinB(pinB) {}

void Encoder::begin()
{
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(pinA, pinB); // Use full quadrature decoding
    encoder.clearCount();
}

float Encoder::getAngle()
{
    long count = encoder.getCount();
    float angle = (360.0 * count) / CPR;
    return angle;
}

void Encoder::reset()
{
    encoder.clearCount();
}
