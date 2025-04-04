#include "IMU.h"
#include <Arduino.h>

IMU::IMU(
    float gx_offset, float gy_offset, float gz_offset,
    float ax_offset, float ay_offset, float az_offset)
    : ax_offset(ax_offset), ay_offset(ay_offset), az_offset(az_offset)
{
}

void IMU::setup()
{
    Wire.begin();
    writeData(PWR_MGMT_1, 0b00000000);   // Wake up MPU6050 (write 0 to power management register)
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

    data.ax_g = ax / ACCEL_SENSITIVITY + ax_offset;
    data.ay_g = ay / ACCEL_SENSITIVITY + ay_offset;
    data.az_g = az / ACCEL_SENSITIVITY + az_offset;

    data.x_deg = atan(data.ay_g / sqrt(data.ax_g * data.ax_g + data.az_g * data.az_g)) * 1 / (3.142 / 180);
    data.y_deg = -atan(data.ax_g / sqrt(data.ay_g * data.ay_g + data.az_g * data.az_g)) * 1 / (3.142 / 180);

    if (debug)
    {
        if (debug)
        {
            Serial.print("Roll: ");
            Serial.print(data.x_deg);
            Serial.print(" | Pitch: ");
            Serial.print(data.y_deg);
            Serial.print(" | Yaw: ");
            Serial.println(0.0); // ou um valor estimado se você quiser tentar calcular o yaw
        }        
    }

    return data;
}
