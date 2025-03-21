#include "imu.h"

void writeData(uint8_t slaveAddr, uint8_t regAddr, uint8_t value) {
    Wire.beginTransmission(slaveAddr);  // Start communication with slave
    Wire.write(regAddr);                // Specify register address
    Wire.write(value);                   // Send data to register
    Wire.endTransmission();              // End transmission
}
