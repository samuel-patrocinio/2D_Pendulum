#ifndef IMU_H
#define IMU_H

#include <Wire.h>

// MPU6050 I²C Address
#define MPU6050_ADDR 0x68  

// MPU6050 Register Addresses
#define MPU6050_PWR_MGMT_1 0x6B  // Power management register
#define MPU6050_GYRO_CONFIG 0x1B // Gyroscope configuration register
#define MPU6050_ACCEL_CONFIG 0x1C // Accelerometer configuration register
#define MPU_ACCEL_VALUES 0x3B // Values of the first accelerometer register

// Function Prototype
void writeData(uint8_t slaveAddr, uint8_t regAddr, uint8_t value);

#endif  // IMU_H
