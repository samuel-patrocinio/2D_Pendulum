// #include "functions.h"
// #include <FastLED.h>

// float k1 = 200;
// float k2 = 10;
// float k3 = 2.5;
// float k4 = 0.0056;

// float pwm_1_1, pwm_1_2, pwm_1_3, pwm_1_4, pwm_2_1, pwm_2_2, pwm_2_3, pwm_2_4, pwm_3_1, pwm_3_2, pwm_3_3, pwm_3_4;

// long currentT = 0;
// long previousT = 0;

// float m1_pwm, m2_pwm, m3_pwm;

// float motor1_angular_position = 0;
// float motor2_angular_position = 0;
// float motor3_angular_position = 0;

// void setup() {
//   Serial.begin(115200);

//   //Setup Bluetooth
//   SerialBT.begin("2d-pendulum");

//   //Setup EEPROM
//   EEPROM.begin(EEPROM_SIZE);

//   //Setup motors
//   setupMotor(ENC1_1, ENC1_2, ENC1_READ, BRAKE1, DIR1, PWM1, PWM1_CH);
//   setupMotor(ENC2_1, ENC2_2, ENC2_READ, BRAKE2, DIR2, PWM2, PWM2_CH);
//   setupMotor(ENC3_1, ENC3_2, ENC3_READ, BRAKE3, DIR3, PWM3, PWM3_CH);

//   Motor1_control(0);
//   Motor2_control(0);
//   Motor3_control(0);

//   //Setup led
//   pinMode(LED, OUTPUT);

//   delay(200);
//   angle_setup();
//   SerialBT.println("Setup done");
// }


// void loop() {
//   currentT = millis();
//   if (currentT - previousT >= loop_time) {
//     previousT = currentT;
//     Tuning();
//     angle_calc();
//     ang_vel_calc();
//     setGains(&k1,&k2,&k3,&k4);

//     pwm_1_1 = k1 * roll_m1;
//     pwm_1_2 = k2 * gyro_m1;
//     pwm_1_3 = k3 * motor1_speed;
//     pwm_1_4 = k4 * motor1_angular_position;

//     pwm_2_1 = k1 * roll_m2;
//     pwm_2_2 = k2 * gyro_m2;
//     pwm_2_3 = k3 * motor2_speed;
//     pwm_2_4 = k4 * motor2_angular_position;

//     pwm_3_1 = k1 * robot_angleX;
//     pwm_3_2 = k2 * gyroXfilt;
//     pwm_3_3 = k3 * motor3_speed;
//     pwm_3_4 = k4 * motor3_angular_position;


//     // M1
//     //Serial.print("M1_1: "); Serial.print(pwm_1_1);Serial.print(" M1_2: "); Serial.print(pwm_1_2);Serial.print(" M1_3: "); Serial.print(pwm_1_3);Serial.print(" M1_4: "); Serial.println(pwm_1_4);
//     // M2
//     //Serial.print("M2_1: "); Serial.print(pwm_2_1);Serial.print(" M2_2: "); Serial.print(pwm_2_2);Serial.print(" M2_3: "); Serial.print(pwm_2_3);Serial.print(" M2_4: "); Serial.println(pwm_2_4);
//     // M3
//     //Serial.print("M3_1: "); Serial.print(pwm_3_1);Serial.print(" M3_2: "); Serial.print(pwm_3_2);Serial.print(" M3_3: "); Serial.print(pwm_3_3);Serial.print(" M3_4: "); Serial.println(pwm_3_4);
//     //General info
//     // Serial.print("B_ang:\t");
//     // Serial.print(body_angle_pwm);
//     // Serial.print("\tB_vel:\t");
//     // Serial.print(body_ang_vel_pwm);
//     // Serial.print("\tW_vel:\t");
//     // Serial.print(wheel_ang_vel_pwm);
//     // Serial.print("\tW_pos:\t");
//     // Serial.print(wheel_ang_pos_pwm);
//     // Serial.print("\tang_X:\t");
//     // Serial.print(robot_angleX);
//     // Serial.print("\tk1:\t");
//     // Serial.print(k1);
//     // Serial.print("\tk2:\t");
//     // Serial.print(k2);
//     // Serial.print("\tk3:\t");
//     // Serial.print(k3);
//     // Serial.print("\tk4:\t");
//     // Serial.print(k4);
//     // Serial.print("\tm3_pwm:\t");
//     // Serial.println(m3_pwm);

//     // Angles
//     // Serial.print("\tm1:\t");
//     // Serial.print(roll_m1);
//     // Serial.print("\tm2:\t");
//     // Serial.print(roll_m2);
//     // Serial.print("\tm3:\t");
//     // Serial.print(robot_angleX);

//     // Gyros
//     // Serial.print("\tm1:\t");
//     // Serial.print(gyro_m1);
//     // Serial.print("\tm2:\t");
//     // Serial.print(gyro_m2);
//     // Serial.print("\tm3:\t");
//     // Serial.println(gyroXfilt);

//     if(is_running){

//       digitalWrite(BRAKE1, HIGH);
//       digitalWrite(BRAKE2, HIGH);
//       digitalWrite(BRAKE3, HIGH);

//       motor1_speed = enc_count1;
//       enc_count1 = 0;
//       motor2_speed = enc_count2;
//       enc_count2 = 0;
//       motor3_speed = enc_count3;
//       enc_count3 = 0;

//       m1_pwm = constrain(pwm_1_1 + pwm_1_2 + pwm_1_3 + pwm_1_4, -255, 255);
//       m2_pwm = constrain(pwm_2_1 + pwm_2_2 + pwm_2_3 + pwm_2_4, -255, 255);
//       m3_pwm = constrain(pwm_3_1 + pwm_3_2 + pwm_3_3 + pwm_3_4, -255, 255);
//       // Integrando a velocidade para obter a posição
//       motor1_angular_position += motor1_speed;
//       motor2_angular_position += motor2_speed;
//       motor3_angular_position += motor3_speed;
//       Motor1_control(m1_pwm);
//       Motor2_control(m2_pwm);
//       Motor3_control(m3_pwm);
//     } else {
//         Motor1_control(0);
//         Motor2_control(0);
//         Motor3_control(0);
//         digitalWrite(BRAKE1, LOW);
//         digitalWrite(BRAKE2, LOW);
//         digitalWrite(BRAKE3, LOW);
//         motor1_angular_position = 0;
//         motor2_angular_position = 0;
//         motor3_angular_position = 0;
//     }
//   }
// }

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

