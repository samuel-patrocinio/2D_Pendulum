#ifndef IMU_H
#define IMU_H

#include <Wire.h>

class IMU {
public:
    struct SensorData {
        float ax_g, ay_g, az_g;
        float gx_dps, gy_dps, gz_dps;
        float temp_C;
    };

    IMU();  // Constructor

    void setup();
    void writeData(uint8_t regAddr, uint8_t value);
    SensorData readSensorData(bool debug = false);

private:
    const uint8_t deviceAddress = 0x68;  // Default I²C address

    // Register addresses (fixed for MPU6050)
    static constexpr uint8_t PWR_MGMT_1     = 0x6B;
    static constexpr uint8_t GYRO_CONFIG    = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG   = 0x1C;
    static constexpr uint8_t ACCEL_VALUES   = 0x3B;

    // Sensitivity scale factors (assuming ±2g, ±250°/s, datasheet values)
    static constexpr float ACCEL_SENSITIVITY = 16384.0;   // LSB/g
    static constexpr float GYRO_SENSITIVITY  = 131.0;     // LSB/(°/s)
    static constexpr float TEMP_SENSITIVITY  = 340.0;     // LSB/°C
    static constexpr float TEMP_OFFSET       = 36.53;     // °C
};

#endif  // IMU_H
