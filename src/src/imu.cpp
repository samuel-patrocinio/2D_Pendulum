#include "IMU.h"
#include <Arduino.h>

IMU::IMU(
    float gx_offset, float gy_offset, float gz_offset,
    float ax_offset, float ay_offset, float az_offset)
    : gx_offset(gx_offset), gy_offset(gy_offset), gz_offset(gz_offset),
      ax_offset(ax_offset), ay_offset(ay_offset), az_offset(az_offset)
{
}

void IMU::setup()
{
    Wire.begin();
    writeData(PWR_MGMT_1, 0b00000000);   // Wake up MPU6050 (write 0 to power management register)
    writeData(GYRO_CONFIG, 0b00001000);  // Set gyroscope range to +/- 500°/s
    writeData(ACCEL_CONFIG, 0b00000000); // Set accelerometer range to +/- 2g
}

void IMU::writeData(uint8_t regAddr, uint8_t value)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddr);
    Wire.write(value);
    Wire.endTransmission();
}

IMU::SensorData IMU::readData(bool debug)
{
    SensorData data;

    Wire.beginTransmission(deviceAddress);
    Wire.write(ACCEL_VALUES); // Start reading from ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddress, (uint8_t)14); // read the next 14 bytes

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    data.ax_g = ax / ACCEL_SENSITIVITY + ax_offset;
    data.ay_g = ay / ACCEL_SENSITIVITY + ay_offset;
    data.az_g = az / ACCEL_SENSITIVITY + az_offset;

    data.temp_C = (temp / TEMP_SENSITIVITY) + TEMP_OFFSET;

    data.gx_dps = gx / GYRO_SENSITIVITY + gx_offset;
    data.gy_dps = gy / GYRO_SENSITIVITY + gy_offset;
    data.gz_dps = gz / GYRO_SENSITIVITY + gz_offset;

    data.x_deg = atan(data.ay_g / sqrt(data.ax_g * data.ax_g + data.az_g * data.az_g)) * 1 / (3.142 / 180);
    data.y_deg = -atan(data.ax_g / sqrt(data.ay_g * data.ay_g + data.az_g * data.az_g)) * 1 / (3.142 / 180);

    if (debug)
    {
        // Serial.print("Acceleration X [g]= ");
        // Serial.print(data.ax_g);
        // Serial.print(" Acceleration Y [g]= ");
        // Serial.print(data.ay_g);
        // Serial.print(" Acceleration Z [g]= ");
        // Serial.println(data.az_g);
        Serial.print("Angle X [deg]= ");
        Serial.print(data.x_deg);
        Serial.print(" Angle Y [deg]= ");
        Serial.println(data.y_deg);
        // Serial.print(" Angle Z [deg]= ");
        // Serial.println(data.z_deg);
    }

    return data;
}
