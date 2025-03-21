#include <Wire.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "imu.h"


// Conversion constants
#define ACCEL_SENSITIVITY 16384.0 // ±2g range, 16384 LSB/g
#define GYRO_SENSITIVITY 131.0    // ±250°/s range, 131 LSB/(°/s)
#define TEMP_SENSITIVITY 340.0    // 340 LSB/°C
#define TEMP_OFFSET 36.53         // Offset for temperature conversion

// Raw sensor values
int16_t ax, ay, az, temp, gx, gy, gz;

// Converted values
float ax_g, ay_g, az_g;       // Acceleration in g
float temp_C;                 // Temperature in Celsius
float gx_dps, gy_dps, gz_dps; // Gyroscope in °/s

ESP32Encoder encoder;

void setup()
{
  
  Serial.begin(115200);
  
  // Initialize I²C
  Wire.begin();

  // Wake up MPU6050 (write 0 to power management register)
  writeData(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0b00000000);

  // Set gyroscope range to +/- 500°/s
  writeData(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0b00001000);

  // Set accelerometer range to +/- 2g
  writeData(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0b00000000);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Attach the encoder pins
  encoder.attachFullQuad(2, 4);

  // Reset the encoder count to 0
  encoder.clearCount();
}

void loop()
{
  // Read sensor data from MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Start reading from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14); // Reads 14 Bytes starting from MPU_ACCEL_VALUES register

  // Read accelerometer data
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  // Read temperature data
  temp = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  // Convert raw values
  ax_g = ax / ACCEL_SENSITIVITY; // Convert to g
  ay_g = ay / ACCEL_SENSITIVITY;
  az_g = az / ACCEL_SENSITIVITY;

  temp_C = (temp / TEMP_SENSITIVITY) + TEMP_OFFSET; // Convert to Celsius

  gx_dps = gx / GYRO_SENSITIVITY; // Convert to degrees per second
  gy_dps = gy / GYRO_SENSITIVITY;
  gz_dps = gz / GYRO_SENSITIVITY;

  //Get encoder values
  long position = encoder.getCount();

  // Print values

  //imu
  Serial.println("--------------------");
  Serial.print("Accel (g): X=");
  Serial.print(ax_g, 2);
  Serial.print(" Y=");
  Serial.print(ay_g, 2);
  Serial.print(" Z=");
  Serial.println(az_g, 2);

  Serial.print("Temp (°C): ");
  Serial.println(temp_C, 2);

  Serial.print("Gyro (°/s): X=");
  Serial.print(gx_dps, 2);
  Serial.print(" Y=");
  Serial.print(gy_dps, 2);
  Serial.print(" Z=");
  Serial.println(gz_dps, 2);

  //Encoder
  Serial.print("Encoder Position: ");
  Serial.println(position);

  delay(500); // Read every 500ms
}
