#include <stdint.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#define BRAKE1      14
#define DIR1        16
#define ENC1_1      25
#define ENC1_2      17
#define PWM1        27
#define PWM1_CH     0
  
#define BRAKE2      18
#define DIR2        23
#define ENC2_1      13
#define ENC2_2      5
#define PWM2        19
#define PWM2_CH     1

#define BRAKE3      15
#define DIR3        32
#define ENC3_1      36
#define ENC3_2      4
#define PWM3        33
#define PWM3_CH     2

#define LED 2

#define TIMER_BIT   8
#define BASE_FREQ   20000

#define MPU6050       0x68   // Device address
#define ACCEL_CONFIG  0x1C   // Accelerometer configuration address
#define GYRO_CONFIG   0x1B   // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
 
#define accSens 0            // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0           // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define EEPROM_SIZE 64

extern int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
extern int loop_time;

extern float Gyro_amount;

extern int16_t  GyZ_offset;
extern int16_t  GyY_offset;
extern int16_t  GyX_offset;

extern float gyro_m1, gyro_m2;
 
extern float robot_angleX, robot_angleY;
extern float roll_m1, roll_m2;
extern float Acc_angleX, Acc_angleY; 

extern long currentT, previousT;

extern bool is_vertical, is_running;

extern volatile long  enc_count1, enc_count2, enc_count3;
extern int state1, state2, state3;

extern int16_t motor1_speed, motor2_speed, motor3_speed;         

extern float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;

struct OffsetsObj {
  int ID;
  float acX;
  float acY;
  float acZ;
};

struct Matrix{
  double m[3][3];
};

extern float k1, k2, k3, k4;

extern BluetoothSerial SerialBT;

void readGyroscope();
void readAccelerometer();
void angle_calc();
void writeTo(byte, byte, byte);
void angle_setup();
int calibrateGyroAxis(char);
int Tuning();
void ENC1_READ();
void ENC2_READ();
void ENC3_READ();
void encoderRead(int pinA, int pinB, volatile long &count, int &state);
void pwmSet(uint8_t channel, uint32_t value);
void Motor1_control(int sp);
void Motor2_control(int sp);
void Motor3_control(int sp);
void setupMotor(int encA, int encB, void (*isr)(), int brakePin, int dirPin, int pwmPin, int pwmChannel);
Matrix matmul(Matrix A, Matrix B);
Matrix euler_to_rotation_matrix(float, float, float);
void printMatrix(Matrix);
void rotation_matrix_to_euler_zyx(Matrix R, float* roll, float* pitch, float* yaw);
void setGains(float *k1, float *k2, float *k3, float *k4);
void ang_vel_calc();

#endif