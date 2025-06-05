#include "functions.h"

long currentT = 0;
long previousT = 0;
long lastCount = 0;
const int pulsesPerRevolution = 400;
float elapsedTime = 0;
float max_voltage = 11.1;
float v;

#define VOLTAGE_TO_PWM(voltage, max_voltage) ((int)((voltage) * 255.0 / (max_voltage)))

void setup(){
  Serial.begin(115200);

  setupMotor(ENC3_1, ENC3_2, ENC3_READ, BRAKE3, DIR3, PWM3, PWM3_CH);

  Motor3_control(0);
}

void loop(){
  currentT = millis();
  if (currentT - previousT >= loop_time) {

    long count = enc_count3;
    long deltaCount = count - lastCount;
    lastCount = count;

    float deltaTime = loop_time / 1000.0;
    float velocity = (deltaCount * 2.0 * PI) / (pulsesPerRevolution * deltaTime);

    elapsedTime += loop_time/1000.0;
    if(elapsedTime >= 0.0 && elapsedTime < 10.0){
      v = 0;
    }
    else if(elapsedTime >= 10.0 && elapsedTime < 20.0){
      v = 2;
    }
    else{
      v = 0;
    }

    Motor3_control(VOLTAGE_TO_PWM(v, max_voltage));

    if(elapsedTime < 36.0){
      Serial.print(elapsedTime); Serial.print(","); Serial.print(v); Serial.print(","); Serial.println(velocity);
    }
    

    previousT = currentT;
  }
}