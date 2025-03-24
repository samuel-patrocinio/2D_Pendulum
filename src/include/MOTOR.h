#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Encoder.h"

class Motor
{
public:
    Motor(gpio_num_t pwmPin, gpio_num_t dirPin, gpio_num_t brakePin, gpio_num_t encA, gpio_num_t encB, uint8_t pwmChannel);

    void begin();
    void setSpeed(int speed); // Positive = CW, Negative = CCW
    void stop();
    float getAngle(bool debug = false);
    void resetPosition();

private:
    gpio_num_t pwmPin, dirPin, brakePin;
    uint8_t pwmChannel;
    Encoder encoder;

    static constexpr int PWM_FREQ = 20000;   // 20 kHz
    static constexpr int PWM_RESOLUTION = 8; // 8-bit (0–255)
};

#endif
