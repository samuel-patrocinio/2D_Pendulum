#include "Encoder.h"

Encoder::Encoder(gpio_num_t pinA, gpio_num_t pinB)
    : pinA(pinA), pinB(pinB) {}

void Encoder::begin() {
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(pinA, pinB);                // Use full quadrature decoding
    encoder.clearCount();
}

long Encoder::getPosition() {
    return encoder.getCount();
}

void Encoder::reset() {
    encoder.clearCount();
}
