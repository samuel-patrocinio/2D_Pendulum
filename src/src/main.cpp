#include <Arduino.h>
#include "IMU.h"
#include "MOTOR.h"

#define DEBUG 1

#define BAULD_RATE 115200
#define LOOP_DELAY 10000


IMU imu;

// ==== Motor 1 Pins ====
gpio_num_t motor1_pwm   = GPIO_NUM_27;
gpio_num_t motor1_dir   = GPIO_NUM_16;
gpio_num_t motor1_brake = GPIO_NUM_14;
gpio_num_t motor1_encA  = GPIO_NUM_25;
gpio_num_t motor1_encB  = GPIO_NUM_17;

// ==== Motor 2 Pins ====
gpio_num_t motor2_pwm   = GPIO_NUM_19;
gpio_num_t motor2_dir   = GPIO_NUM_23;
gpio_num_t motor2_brake = GPIO_NUM_18;
gpio_num_t motor2_encA  = GPIO_NUM_13;
gpio_num_t motor2_encB  = GPIO_NUM_5;

// ==== Motor 3 Pins ====
gpio_num_t motor3_pwm   = GPIO_NUM_33;
gpio_num_t motor3_dir   = GPIO_NUM_32;
gpio_num_t motor3_brake = GPIO_NUM_15;
gpio_num_t motor3_encA  = GPIO_NUM_2;
gpio_num_t motor3_encB  = GPIO_NUM_4;

// ==== Motor PWM Channels ====
uint8_t motor1_pwm_ch = 0;
uint8_t motor2_pwm_ch = 1;
uint8_t motor3_pwm_ch = 2;

// ==== Motor Objects ====
Motor motor_1(motor1_pwm, motor1_dir, motor1_brake, motor1_encA, motor1_encB, motor1_pwm_ch);
Motor motor_2(motor2_pwm, motor2_dir, motor2_brake, motor2_encA, motor2_encB, motor2_pwm_ch);
Motor motor_3(motor3_pwm, motor3_dir, motor3_brake, motor3_encA, motor3_encB, motor3_pwm_ch);

void setup() {
    Serial.begin(BAULD_RATE);

    imu.setup();

    motor_1.begin();
    motor_2.begin();
    motor_3.begin();
}

void loop() {

    IMU::SensorData data = imu.readSensorData(DEBUG);

    long m1_pos = motor_1.getPosition(DEBUG);
    long m2_pos = motor_2.getPosition(DEBUG);
    long m3_pos = motor_3.getPosition(DEBUG);

    Serial.println("Speed 10%");
    motor_1.setSpeed(10);  // 10%
    motor_2.setSpeed(10);  // 10%
    motor_3.setSpeed(10);  // 10%
    delay(LOOP_DELAY);
    Serial.println("Speed 50%");
    motor_1.setSpeed(50);  // 50%
    motor_2.setSpeed(50);  // 50%
    motor_3.setSpeed(50);  // 50%
    delay(LOOP_DELAY);
    Serial.println("Speed 100%");
    motor_1.setSpeed(100);  // 100%
    motor_2.setSpeed(100);  // 100%
    motor_3.setSpeed(100);  // 100%
    delay(LOOP_DELAY);
    Serial.println("Speed 0%");
    motor_1.setSpeed(0);  // 0%
    motor_2.setSpeed(0);  // 0%
    motor_3.setSpeed(0);  // 0%
    delay(LOOP_DELAY);
}
