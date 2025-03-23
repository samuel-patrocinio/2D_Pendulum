#include "Motor.h"

#define PWM_FREQ 20000     // 20 kHz
#define PWM_RESOLUTION 8   // 8-bit (0–255)

Motor::Motor(gpio_num_t pwmPin, gpio_num_t dirPin, gpio_num_t brakePin,
             gpio_num_t encA, gpio_num_t encB, uint8_t pwmChannel)
    : pwmPin(pwmPin), dirPin(dirPin), brakePin(brakePin), pwmChannel(pwmChannel), encoder(encA, encB) {}

void Motor::begin() {
    pinMode((uint8_t)dirPin, OUTPUT);
    pinMode((uint8_t)brakePin, OUTPUT);

    // Setup inverted PWM
    ledcSetup(pwmChannel, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(pwmChannel, 255);                     // Inverted: 255 = 0% duty (motor OFF)
    ledcAttachPin((uint8_t)pwmPin, pwmChannel);

    // Apply brake (ACTIVE LOW)
    digitalWrite((uint8_t)brakePin, LOW);

    encoder.begin();
}

void Motor::setSpeed(int percentage) {
    // Clamp to 0–100
    percentage = constrain(percentage, -100, 100);

    // Release brake (ACTIVE LOW = OFF = HIGH)
    digitalWrite((uint8_t)brakePin, HIGH);

    // Set direction
    if (percentage >= 0) {
        digitalWrite((uint8_t)dirPin, HIGH);
    } else {
        digitalWrite((uint8_t)dirPin, LOW);
        percentage = -percentage;
    }

    // Convert percentage to inverted 8-bit PWM value
    uint8_t duty = map(percentage, 0, 100, 255, 0);  // 100% = 0, 0% = 255
    ledcWrite(pwmChannel, duty);
}

void Motor::stop() {
    ledcWrite(pwmChannel, 255);                     // Inverted: 255 = 0% duty (motor OFF)
    digitalWrite((uint8_t)brakePin, LOW);           // ACTIVE LOW = brake ON
}

long Motor::getPosition(bool debug) {
    long pos = encoder.getPosition();
    if (debug) {
        Serial.print("Motor @ PWM channel ");
        Serial.print(pwmChannel);
        Serial.print(" → Position: ");
        Serial.println(pos);
    }
    return pos;
}

void Motor::resetPosition() {
    encoder.reset();
}
