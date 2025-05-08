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

 
#define TIMER_BIT   8
#define BASE_FREQ   20000
 
#define MPU6050       0x68   // Device address
#define ACCEL_CONFIG  0x1C   // Accelerometer configuration address
#define GYRO_CONFIG   0x1B   // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
 
#define accSens 0            // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0           // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
 
#define EEPROM_SIZE   64
 
#define NUM_PIXELS    3      // The number of LEDs (pixels) on WS2812B
 
float Gyro_amount = 0.996;
 
bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;
 
float K1 = 180;
float K2 = 28.00;
float K3 = 8;
float K4 = 0.064;
float zK2 = 8.00;
float zK3 = 0.30;
 
float eK1 = 190;
float eK2 = 31.00;
float eK3 = 2.5;
float eK4 = 0.014;
 
int loop_time = 15;
 
struct OffsetsObj {
  int ID;
  float acXv;
  float acYv;
  float acZv;
  float acXe;
  float acYe;
  float acZe;
};
OffsetsObj offsets;
 
float alpha = 0.7;
 
int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;
 
int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;
int32_t  GyX_offset_sum = 0;
 
float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;      
int32_t motors_speed_X;
int32_t motors_speed_Y;
int32_t motors_speed_Z;   
 
long currentT, previousT_1, previousT_2;
 
volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;         
int16_t motor2_speed;         
int16_t motor3_speed;     