#include "defs.h"
#include <EEPROM.h>
#include <Wire.h>

#define BAULD_RATE 115200

void setup() {
    Serial.begin(BAULD_RATE);
    EEPROM.begin(EEPROM_SIZE);

    pinMode(BRAKE1, OUTPUT);
    pinMode(BRAKE2, OUTPUT);
    pinMode(BRAKE3, OUTPUT);
   
    digitalWrite(BRAKE1, HIGH);
    digitalWrite(BRAKE2, HIGH);
    digitalWrite(BRAKE3, HIGH);
    
    pinMode(DIR1, OUTPUT);
    pinMode(ENC1_1, INPUT);
    pinMode(ENC1_2, INPUT);
    attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
    attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
    ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM1, PWM1_CH);
    Motor1_control(0);
    
    pinMode(DIR2, OUTPUT);
    pinMode(ENC2_1, INPUT);
    pinMode(ENC2_2, INPUT);
    attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
    attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
    ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM2, PWM2_CH);
    Motor2_control(0);
    
    pinMode(DIR3, OUTPUT);
    pinMode(ENC3_1, INPUT);
    pinMode(ENC3_2, INPUT);
    attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
    attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
    ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(PWM3, PWM3_CH);
    Motor3_control(0);
   
    EEPROM.get(0, offsets);
    if (offsets.ID == 96)
      calibrated = true;
   
    delay(200);
    angle_setup();
}

void loop() {

}