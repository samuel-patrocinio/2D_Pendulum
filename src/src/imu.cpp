#include "IMU.h"
#include <Arduino.h>

IMU::IMU() {}

void IMU::setup(){
    Wire.begin();
    writeData(PWR_MGMT_1, 0b00000000);  // Wake up MPU6050 (write 0 to power management register)
    writeData(GYRO_CONFIG, 0b00001000);  // Set gyroscope range to +/- 500°/s
    writeData(ACCEL_CONFIG, 0b00000000);  // Set accelerometer range to +/- 2g
}

void IMU::writeData(uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddr);
    Wire.write(value);
    Wire.endTransmission();
}

IMU::SensorData IMU::readSensorData(bool debug) {
    SensorData data;

    Wire.beginTransmission(deviceAddress);
    Wire.write(ACCEL_VALUES); // Start reading from ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddress, (uint8_t)14);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    data.ax_g = ax / ACCEL_SENSITIVITY;
    data.ay_g = ay / ACCEL_SENSITIVITY;
    data.az_g = az / ACCEL_SENSITIVITY;

    data.temp_C = (temp / TEMP_SENSITIVITY) + TEMP_OFFSET;

    data.gx_dps = gx / GYRO_SENSITIVITY;
    data.gy_dps = gy / GYRO_SENSITIVITY;
    data.gz_dps = gz / GYRO_SENSITIVITY;

    if (debug) {
        Serial.println("----- IMU Raw Data -----");
        Serial.print("Accel Raw: ax="); Serial.print(ax);
        Serial.print(" ay="); Serial.print(ay);
        Serial.print(" az="); Serial.println(az);

        Serial.print("Temp Raw: "); Serial.println(temp);

        Serial.print("Gyro Raw: gx="); Serial.print(gx);
        Serial.print(" gy="); Serial.print(gy);
        Serial.print(" gz="); Serial.println(gz);

        Serial.println("----- IMU Converted Data -----");
        Serial.print("Accel (g): ax="); Serial.print(data.ax_g, 3);
        Serial.print(" ay="); Serial.print(data.ay_g, 3);
        Serial.print(" az="); Serial.println(data.az_g, 3);

        Serial.print("Temp (°C): "); Serial.println(data.temp_C, 2);

        Serial.print("Gyro (°/s): gx="); Serial.print(data.gx_dps, 3);
        Serial.print(" gy="); Serial.print(data.gy_dps, 3);
        Serial.print(" gz="); Serial.println(data.gz_dps, 3);

        Serial.println();
    }

    return data;
}