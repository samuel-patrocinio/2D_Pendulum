#ifndef IMU_H
#define IMU_H

#include <Wire.h>

class IMU
{
public:
    struct SensorData
    {
        float ax_g, ay_g, az_g;
        float x_deg, y_deg, z_deg;
    };

    IMU(
        float gx_offset = 0, float gy_offset = 0, float gz_offset = 0,
        float ax_offset = 0, float ay_offset = 0, float az_offset = 0);

    void setup();
    void writeData(uint8_t regAddr, uint8_t value);
    SensorData readData(bool debug = false);

private:
    const uint8_t deviceAddress = 0x68; // Default I²C address

    // Register addresses (fixed for MPU6050)
    static constexpr uint8_t PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t ACCEL_VALUES = 0x3B;

    // Sensitivity scale factors (assuming ±2g, ±500°/s, datasheet values)
    static constexpr float ACCEL_SENSITIVITY = 16384.0; // LSB/g

    float ax_offset, ay_offset, az_offset;
};

#endif // IMU_H
