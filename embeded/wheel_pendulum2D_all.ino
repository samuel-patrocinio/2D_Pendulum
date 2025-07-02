// I2C libray communication
#include <Wire.h>

// ENCODER library based on the built in counter hardware
#include "ESP32Encoder.h"

// Bluetooth communication
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

//EEPROM
#include <EEPROM.h>

// ESP32 BLUE LED pin
#define INTERNAL_LED 2

// IMU I2C address
#define MPU   0x68

// NIDEC PWM config
#define NIDEC_TIMER_BIT   8
#define NIDEC_BASE_FREQ   20000

//EEPROM flags
#define EEPROM_VALID_FLAG 100  // Endereço do byte de validação
#define EEPROM_START_ADDR 101  // Início real dos dados

// NIDEC M1 pins
#define M1_BRAKE            14 //Yellow wire (Start/Stop)
#define M1_NIDEC_PWM        27 //Write wire  (PWM)
#define M1_DIR              16 //Green wire  (Forward/Reverse) 
#define M1_ENCA             17 //Brown wire  (Signal A)  
#define M1_ENCB             25 //Orange      (Signal B)  
#define M1_NIDEC_PWM_CH      0

// NIDEC M2 pins
#define M2_BRAKE            18 //Yellow wire (Start/Stop)
#define M2_NIDEC_PWM        19 //Write wire  (PWM)
#define M2_DIR              23 //Green wire  (Forward/Reverse) 
#define M2_ENCA             5 //Brown wire  (Signal A)  
#define M2_ENCB             13 //Orange      (Signal B)  
#define M2_NIDEC_PWM_CH      1

// // NIDEC M3 pins
#define M3_BRAKE            15 //Yellow wire (Start/Stop)
#define M3_NIDEC_PWM        33 //Write wire  (PWM)
#define M3_DIR              32 //Green wire  (Forward/Reverse) 
#define M3_ENCA             4 //Brown wire  (Signal A)  
#define M3_ENCB             36 //Orange      (Signal B)  
#define M3_NIDEC_PWM_CH      2

// Encoder vars
ESP32Encoder M1_NIDEC_ENC;
ESP32Encoder M2_NIDEC_ENC;
ESP32Encoder M3_NIDEC_ENC;

// Kalman Filter vars
float Q_angle = 0.001; // Angular data confidence
float Q_bias  = 0.005; // Angular velocity data confidence
float R_meas  = 1.0;

float M1_roll_angle = 0.0;
float M2_roll_angle = 0.0;
float M3_roll_angle = 0.0;

float M1_bias = 0.0;
float M2_bias = 0.0;
float M3_bias = 0.0;

float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float K[2] = {0, 0};

// Control vars
//     theta         dthehta         omega           integral
// Control that works for 1D
// float M1_K1 = -450,    M1_K2 = -26.30,    M1_K3 = -0.462,    M1_K4 = 0.651;
// float M2_K1 = -450,    M2_K2 = -26.30,    M2_K3 = -0.462,    M2_K4 = 0.651;
// float M3_K1 = -450,    M3_K2 = -26.30,    M3_K3 = -0.462,    M3_K4 = 0.651;
// Control that works for 2D
// float M1_K1 = -490,    M1_K2 = -35.00,    M1_K3 = -0.462,    M1_K4 = 0.651;
// float M2_K1 = -490,    M2_K2 = -35.00,    M2_K3 = -0.462,    M2_K4 = 0.651;
// float M3_K1 = -490,    M3_K2 = -35.00,    M3_K3 = -0.462,    M3_K4 = 0.651;
//tests
float M1_K1 = -450,    M1_K2 = -26.30,    M1_K3 = -0.462,    M1_K4 = 0.651;
float M2_K1 = -450,    M2_K2 = -26.30,    M2_K3 = -0.462,    M2_K4 = 0.651;
float M3_K1 = -450,    M3_K2 = -26.30,    M3_K3 = -0.462,    M3_K4 = 0.651;


float M1_U = 0;
float M2_U = 0;
float M3_U = 0;

int M1_pwm = 0;
int M2_pwm = 0;
int M3_pwm = 0;

float M1_theta = 0.0, M1_theta_dot = 0.0;            // System states
float M2_theta = 0.0, M2_theta_dot = 0.0;            // System states
float M3_theta = 0.0, M3_theta_dot = 0.0;            // System states

float M1_omega = 0, M1_integral = 0, M1_integral_past = 0;
float M2_omega = 0, M2_integral = 0, M2_integral_past = 0;
float M3_omega = 0, M3_integral = 0, M3_integral_past = 0;

float Ts = 0.01, currentT = 0.0, previousT = 0.0;        // Elapsed time in loop() function

bool M1_ENABLE = false;
bool M2_ENABLE = false;
bool M3_ENABLE = false;

bool is_running = false;

unsigned long motor2_start_time = 0;
bool motor2_active = false;

// Calibração da IMU
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

void IMUcalibrate(int samples = 500) {
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  SerialBT.println("Calibrando IMU...");

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // temperature (ignorado)
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(5);
  }

  ax_offset = ax_sum / (float)samples;
  ay_offset = ay_sum / (float)samples;
  az_offset = (az_sum / (float)samples) - 16384.0;  // compensar gravidade em Z se necessário
  gx_offset = gx_sum / (float)samples;
  gy_offset = gy_sum / (float)samples;
  gz_offset = gz_sum / (float)samples;

  SerialBT.println("Calibração completa.");
}

// MAIN SETUP
void setup() { // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);                   // make sure your Serial Monitor is also set at this baud rate.
  SerialBT.begin("2DPendulum"); // Bluetooth device name

  pinMode(INTERNAL_LED,OUTPUT);
  digitalWrite(INTERNAL_LED,HIGH);  // Turn on blue led

  EEPROM.begin(128);

  NIDECsetup();
  IMUsetup();
  delay(1000);
  IMUcalibrate();

  delay(1500);                      // Wait for the system to stabilize
  for (int i=1; i<= 400; i++){      // Wait for the Kalman filter stabilize
    IMUread();
    delay(5);
  }
  currentT = millis();
  digitalWrite(INTERNAL_LED,LOW);
}

// MAIN LOOP
void loop() {// put your main code here, to run repeatedly:

  if (motor2_active && (millis() - motor2_start_time >= 5000)) {
    M2_MOTORcmd(0);
    motor2_active = false;
  }

  currentT = millis();
  if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;
    Tuning();                   // tuning control parameters
    IMUread();

    if ( abs(M1_roll_angle) < 15 && abs(M2_roll_angle) < 15 && abs(M3_roll_angle) < 15 ){

      enable_all_motors();

      digitalWrite(INTERNAL_LED,LOW); 

      digitalWrite(M1_BRAKE, HIGH);
      digitalWrite(M2_BRAKE, HIGH);
      digitalWrite(M3_BRAKE, HIGH);

      M1_theta += M1_theta_dot * Ts;
      M2_theta += M2_theta_dot * Ts;
      M3_theta += M3_theta_dot * Ts;

      M1_integral = -M1_NIDEC_ENC.getCount()/63.7;
      M2_integral = -M2_NIDEC_ENC.getCount()/63.7;
      M3_integral = -M3_NIDEC_ENC.getCount()/63.7;

      M1_omega = -(M1_integral - M1_integral_past)/Ts;
      M2_omega = -(M2_integral - M2_integral_past)/Ts;
      M3_omega = -(M3_integral - M3_integral_past)/Ts;

      M1_integral_past = M1_integral;
      M2_integral_past = M2_integral;
      M3_integral_past = M3_integral;

      M1_U = -(M1_K1*M1_theta + M1_K2*M1_theta_dot + M1_K3*M1_omega + M1_K4*M1_integral);
      M2_U = -(M2_K1*M2_theta + M2_K2*M2_theta_dot + M2_K3*M2_omega + M2_K4*M2_integral);
      M3_U = -(M3_K1*M3_theta + M3_K2*M3_theta_dot + M3_K3*M3_omega + M3_K4*M3_integral);

      M1_pwm = M1_U * 21.3 * M1_ENABLE;
      M2_pwm = M2_U * 21.3 * M2_ENABLE;
      M3_pwm = M3_U * 21.3 * M3_ENABLE;

      SerialBT.print("M1: ");SerialBT.print(M1_theta);
      SerialBT.print(" M2: ");SerialBT.print(M2_theta);
      SerialBT.print(" M3: ");SerialBT.print(M3_theta);
      SerialBT.print(" pwm1: ");SerialBT.print(M1_U * 21.3);
      SerialBT.print(" pwm2: ");SerialBT.print(M2_U * 21.3);
      SerialBT.print(" pwm3: ");SerialBT.println(M3_U * 21.3);

      M1_MOTORcmd(M1_pwm/1.732);
      M2_MOTORcmd(-M1_pwm/1.732);
      M3_MOTORcmd(M3_pwm);

      float M1_voltage = M1_pwm * 11.1 / (255.0 * 1.732);
      float M2_voltage = -M1_pwm * 11.1 / (255.0 * 1.732);
      float M3_voltage = M3_pwm * 11.1 / (255.0);
      float t = previousT / 1000.0; // tempo em segundos (com casas decimais)
      
      Serial.print(t, 3);         // imprime tempo com 3 casas decimais
      Serial.print(",");
      Serial.print(M1_voltage, 2);
      Serial.print(",");
      Serial.print(M2_voltage, 2);
      Serial.print(",");
      Serial.print(M3_voltage, 2);
      Serial.print(",");
      Serial.print(M1_theta, 4);
      Serial.print(",");
      Serial.print(M3_theta, 4);
      Serial.print(",");
      Serial.print(M1_theta_dot, 4);
      Serial.print(",");
      Serial.print(M3_theta_dot, 4);
      Serial.print(",");
      Serial.print(M1_omega, 4);
      Serial.print(",");
      Serial.print(M2_omega, 4);
      Serial.print(",");
      Serial.println(M3_omega, 4);
      
    } else {
      digitalWrite(M1_BRAKE, LOW); // stop reaction wheel
      digitalWrite(M2_BRAKE, LOW); // stop reaction wheel
      digitalWrite(M3_BRAKE, LOW); // stop reaction wheel

      digitalWrite(INTERNAL_LED,HIGH);

      M1_ENABLE = false;
      M2_ENABLE = false;
      M3_ENABLE = false;

      is_running = false;

      for (int i=1; i<= 400; i++){ //Wait for the Kalman Filter stabilize
        IMUread();
        delay(5);
      }
      previousT = millis();
      M1_theta = 0.0;
      M2_theta = 0.0;
      M3_theta = 0.0;

      M1_integral = 0.0;
      M2_integral = 0.0;
      M3_integral = 0.0;

      M1_NIDEC_ENC.clearCount();
      M2_NIDEC_ENC.clearCount();
      M3_NIDEC_ENC.clearCount();
    }
  }   

}

// SETUP functions
void IMUsetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                      //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                      //We want to write to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00000000);             //Set the register bits as 00000000 (250dps full scale), 00010000 (1000dps full scale)
  Wire.write(1 << 3);
  Wire.endTransmission();                //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU);           //Start communication with the address found during search.
  Wire.write(0x1C);                      //We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                //Set the register bits as 00000000 (+/- 2g full scale range), 00010000 (+/- 8g full scale range)
  Wire.endTransmission(); 
}

void NIDECsetup(){
  // M1 setup
  pinMode(M1_BRAKE, OUTPUT);
  digitalWrite(M1_BRAKE, HIGH);

  pinMode(M1_DIR, OUTPUT);
  ledcSetup(M1_NIDEC_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(M1_NIDEC_PWM, M1_NIDEC_PWM_CH);
  M1_MOTORcmd(0);
  
  // Encoder setup:
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Encoder:
	M1_NIDEC_ENC.attachFullQuad(M1_ENCB, M1_ENCA);
  M1_NIDEC_ENC.clearCount();

  // M2 setup
  pinMode(M2_BRAKE, OUTPUT);
  digitalWrite(M2_BRAKE, HIGH);

  pinMode(M2_DIR, OUTPUT);
  ledcSetup(M2_NIDEC_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(M2_NIDEC_PWM, M2_NIDEC_PWM_CH);
  M2_MOTORcmd(0);
  
  // Encoder setup:
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Encoder:
	M2_NIDEC_ENC.attachFullQuad(M2_ENCB, M2_ENCA);
  M2_NIDEC_ENC.clearCount();

  // M3 setup
  pinMode(M3_BRAKE, OUTPUT);
  digitalWrite(M3_BRAKE, HIGH);

  pinMode(M3_DIR, OUTPUT);
  ledcSetup(M3_NIDEC_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(M3_NIDEC_PWM, M3_NIDEC_PWM_CH);
  M3_MOTORcmd(0);
  
  // Encoder setup:
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Encoder:
	M3_NIDEC_ENC.attachFullQuad(M3_ENCB, M3_ENCA);
  M3_NIDEC_ENC.clearCount();
}

// IMU function: Kalman Filter
void IMUread(){
  // read IMU
  int16_t M1_ax,M1_ay,M1_az,M1_temp,M1_gx,M1_gy,M1_gz;
  int16_t M2_ax,M2_ay,M2_az,M2_temp,M2_gx,M2_gy,M2_gz;
  int16_t M3_ax,M3_ay,M3_az,M3_temp,M3_gx,M3_gy,M3_gz;
  int16_t ax_temp, ay_temp, az_temp, temp_temp, gx_temp, gy_temp, gz_temp;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);        // IMU address: 0x68
  ax_temp = (Wire.read() << 8 | Wire.read()) - ax_offset;
  ay_temp = (Wire.read() << 8 | Wire.read()) - ay_offset;
  az_temp = (Wire.read() << 8 | Wire.read()) - az_offset;
  temp_temp=Wire.read()<<8|Wire.read();      
  gx_temp = (Wire.read() << 8 | Wire.read()) - gx_offset;
  gy_temp = (Wire.read() << 8 | Wire.read()) - gy_offset;
  gz_temp = (Wire.read() << 8 | Wire.read()) - gz_offset;

  // M1
  M1_ax = 1/2 * ax_temp - 0.866 * ay_temp;
  M1_gx = 1/2 * gx_temp - 0.866 * gy_temp;
  M1_ay = 0.866 * ax_temp + 1/2 * ay_temp;
  M1_gy = 0.866 * gx_temp + 1/2 * gy_temp;
  M1_az = az_temp;
  M1_gz = gz_temp;


  // M2
  M2_ax = 1/2 * ax_temp + 0.866 * ay_temp;
  M2_gx = 1/2 * gx_temp + 0.866 * gy_temp;
  M2_ay = -0.866 * ax_temp + 1/2 * ay_temp;
  M2_gy = -0.866 * gx_temp + 1/2 * gy_temp;
  M2_az = az_temp;
  M2_gz = gz_temp;

  // M3
  M3_ax = -ax_temp;
  M3_gx = -gx_temp;
  M3_ay = -ay_temp;
  M3_gy = -gy_temp;
  M3_az = az_temp;
  M3_gz = gz_temp;

  float M1_ax_angle = atan2(M1_ay, sqrt(M1_ax*M1_ax + M1_az*M1_az)) * 57.3;
  float M2_ax_angle = atan2(M2_ay, sqrt(M2_ax*M2_ax + M2_az*M2_az)) * 57.3;
  float M3_ax_angle = atan2(M3_ay, sqrt(M3_ax*M3_ax + M3_az*M3_az)) * 57.3;

  M1_gx =  M1_gx / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  M2_gx =  M2_gx / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  M3_gx =  M3_gx / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  
  // begin: Kalman filter - Roll Axis (X)
  M1_roll_angle += (M1_gx - M1_bias) * Ts;
  M2_roll_angle += (M2_gx - M2_bias) * Ts;
  M3_roll_angle += (M3_gx - M3_bias) * Ts;
  
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;
  //
  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);  
  //

  M1_roll_angle += K[0] * (M1_ax_angle - M1_roll_angle);
  M2_roll_angle += K[0] * (M2_ax_angle - M2_roll_angle); 
  M3_roll_angle += K[0] * (M3_ax_angle - M3_roll_angle);

  M1_bias += K[1] * (M1_ax_angle - M1_roll_angle);
  M2_bias += K[1] * (M2_ax_angle - M2_roll_angle);  
  M3_bias += K[1] * (M3_ax_angle - M3_roll_angle);
  //
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  // end: Kalman filter 
  
  M1_theta_dot = (M1_gx - M1_bias)/57.3;
  M2_theta_dot = (M2_gx - M2_bias)/57.3;
  M3_theta_dot = (M3_gx - M3_bias)/57.3;

}  

// NIDEC functions
void M1_MOTORcmd(int sp) {
  if (sp < 0) {
    digitalWrite(M1_DIR, HIGH);
    sp = -sp;
  } else {
    digitalWrite(M1_DIR, LOW);
  }
  ledcWrite(M1_NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

void M2_MOTORcmd(int sp) {
  if (sp < 0) {
    digitalWrite(M2_DIR, HIGH);
    sp = -sp;
  } else {
    digitalWrite(M2_DIR, LOW);
  }
  ledcWrite(M2_NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

void M3_MOTORcmd(int sp) {
  if (sp < 0) {
    digitalWrite(M3_DIR, HIGH);
    sp = -sp;
  } else {
    digitalWrite(M3_DIR, LOW);
  }
  ledcWrite(M3_NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

//TUNING function
int Tuning() {
  if (!SerialBT.available())  return 0;
  char motor = SerialBT.read();               // get motor byte
  if (motor == 's'){
    saveGainsToEEPROM();
    loadGainsFromEEPROM();
    print_values();
    return 1;
  }
  if (!SerialBT.available()) return 0;
  char gain = SerialBT.read();                 // get gain byte
  if (motor == 'd' && (gain == '+' || gain == '-')) {
    int sp = (gain == '+') ? 120 : -120;
    M2_MOTORcmd(sp);
    motor2_start_time = millis();
    motor2_active = true;
    return 1;
  }
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get cmd byte
  switch (motor) {
    case '1':
      if      (gain == 'a') M1_K1 += (cmd == '+') ? 2    : (cmd == '-') ? -2    : 0;
      else if (gain == 'b') M1_K2 += (cmd == '+') ? 0.5  : (cmd == '-') ? -0.5  : 0;
      else if (gain == 'c') M1_K3 += (cmd == '+') ? 0.01 : (cmd == '-') ? -0.01 : 0;
      else if (gain == 'd') M1_K4 += (cmd == '+') ? 0.01 : (cmd == '-') ? -0.01 : 0;
      else if (gain == 'e') M1_ENABLE = (cmd == '+');
      saveGainsToEEPROM();
      loadGainsFromEEPROM();
      print_values();
      break;
    case '2':
      if      (gain == 'a') M2_K1 += (cmd == '+') ? 2    : (cmd == '-') ? -2    : 0;
      else if (gain == 'b') M2_K2 += (cmd == '+') ? 0.5  : (cmd == '-') ? -0.5  : 0;
      else if (gain == 'c') M2_K3 += (cmd == '+') ? 0.01 : (cmd == '-') ? -0.01 : 0;
      else if (gain == 'd') M2_K4 += (cmd == '+') ? 0.01 : (cmd == '-') ? -0.01 : 0;
      else if (gain == 'e') M2_ENABLE = (cmd == '+');
      saveGainsToEEPROM();
      loadGainsFromEEPROM();
      print_values();
      break;
    case '3':
      if      (gain == 'a') M3_K1 += (cmd == '+') ? 2    : (cmd == '-') ? -2    : 0;
      else if (gain == 'b') M3_K2 += (cmd == '+') ? 0.5  : (cmd == '-') ? -0.5  : 0;
      else if (gain == 'c') M3_K3 += (cmd == '+') ? 0.01 : (cmd == '-') ? -0.01 : 0;
      else if (gain == 'd') M3_K4 += (cmd == '+') ? 0.01 : (cmd == '-') ? -0.01 : 0;
      else if (gain == 'e') M3_ENABLE = (cmd == '+');
      saveGainsToEEPROM();
      loadGainsFromEEPROM();
      print_values();
      break;
  }
  return 1;  
}

void print_values() {
  SerialBT.print("M1_K1(a): "); SerialBT.print(M1_K1);
  SerialBT.print(" M1_K2(b): "); SerialBT.print(M1_K2);
  SerialBT.print(" M1_K3(c): "); SerialBT.print(M1_K3,4);
  SerialBT.print(" M1_K4(d): "); SerialBT.print(M1_K4,4);
  SerialBT.print(" M1_ENABLE(e): "); SerialBT.println(M1_ENABLE ? "1" : "0");


  SerialBT.print("M2_K1(a): "); SerialBT.print(M2_K1);
  SerialBT.print(" M2_K2(b): "); SerialBT.print(M2_K2);
  SerialBT.print(" M2_K3(c): "); SerialBT.print(M2_K3,4);
  SerialBT.print(" M2_K4(d): "); SerialBT.print(M2_K4,4);
  SerialBT.print(" M2_ENABLE(e): "); SerialBT.println(M2_ENABLE ? "1" : "0");

  SerialBT.print("M3_K1(a): "); SerialBT.print(M3_K1);
  SerialBT.print(" M3_K2(b): "); SerialBT.print(M3_K2);
  SerialBT.print(" M3_K3(c): "); SerialBT.print(M3_K3,4);
  SerialBT.print(" M3_K4(d): "); SerialBT.print(M3_K4,4);
  SerialBT.print(" M3_ENABLE(e): "); SerialBT.println(M3_ENABLE ? "1" : "0");
}

void enable_all_motors(){
  if ( !is_running ){
    int count = M1_NIDEC_ENC.getCount();
    if ( count <= -400 ){
      digitalWrite(INTERNAL_LED,HIGH);
      delay(3000);
      digitalWrite(INTERNAL_LED,LOW);
      M1_NIDEC_ENC.clearCount();
      M2_NIDEC_ENC.clearCount();
      M3_NIDEC_ENC.clearCount();
      is_running = true;
      M1_ENABLE = true;
      M2_ENABLE = true;
      M3_ENABLE = true;
      M1_theta = 0.0, M1_theta_dot = 0.0;            
      M2_theta = 0.0, M2_theta_dot = 0.0;            
      M3_theta = 0.0, M3_theta_dot = 0.0;
      M1_omega = 0, M1_integral = 0, M1_integral_past = 0;
      M2_omega = 0, M2_integral = 0, M2_integral_past = 0;
      M3_omega = 0, M3_integral = 0, M3_integral_past = 0;
    }
    else if ( count >= 400 ) {
      loadGainsFromEEPROM();
      digitalWrite(INTERNAL_LED,HIGH);
      delay(3000);
      digitalWrite(INTERNAL_LED,LOW);
      M1_NIDEC_ENC.clearCount();
      M2_NIDEC_ENC.clearCount();
      M3_NIDEC_ENC.clearCount();
      is_running = true;
      M1_ENABLE = true;
      M2_ENABLE = true;
      M3_ENABLE = true;
      M1_theta = 0.0, M1_theta_dot = 0.0;            
      M2_theta = 0.0, M2_theta_dot = 0.0;            
      M3_theta = 0.0, M3_theta_dot = 0.0;
      M1_omega = 0, M1_integral = 0, M1_integral_past = 0;
      M2_omega = 0, M2_integral = 0, M2_integral_past = 0;
      M3_omega = 0, M3_integral = 0, M3_integral_past = 0;
    }

  }
}

void saveGainsToEEPROM() {
  EEPROM.write(EEPROM_VALID_FLAG, 0xAB); // Byte de verificação

  int addr = EEPROM_START_ADDR;
  EEPROM.put(addr, M1_K1); addr += 4;
  EEPROM.put(addr, M1_K2); addr += 4;
  EEPROM.put(addr, M1_K3); addr += 4;
  EEPROM.put(addr, M1_K4); addr += 4;
  EEPROM.put(addr, M2_K1); addr += 4;
  EEPROM.put(addr, M2_K2); addr += 4;
  EEPROM.put(addr, M2_K3); addr += 4;
  EEPROM.put(addr, M2_K4); addr += 4;
  EEPROM.put(addr, M3_K1); addr += 4;
  EEPROM.put(addr, M3_K2); addr += 4;
  EEPROM.put(addr, M3_K3); addr += 4;
  EEPROM.put(addr, M3_K4);

  EEPROM.commit();
  SerialBT.println("Ganhos salvos na EEPROM.");
}

void loadGainsFromEEPROM() {
  if (EEPROM.read(EEPROM_VALID_FLAG) != 0xAB) {
    SerialBT.println("Nenhum dado válido encontrado na EEPROM.");
    return;
  }

  int addr = EEPROM_START_ADDR;
  EEPROM.get(addr, M1_K1); addr += 4;
  EEPROM.get(addr, M1_K2); addr += 4;
  EEPROM.get(addr, M1_K3); addr += 4;
  EEPROM.get(addr, M1_K4); addr += 4;
  EEPROM.get(addr, M2_K1); addr += 4;
  EEPROM.get(addr, M2_K2); addr += 4;
  EEPROM.get(addr, M2_K3); addr += 4;
  EEPROM.get(addr, M2_K4); addr += 4;
  EEPROM.get(addr, M3_K1); addr += 4;
  EEPROM.get(addr, M3_K2); addr += 4;
  EEPROM.get(addr, M3_K3); addr += 4;
  EEPROM.get(addr, M3_K4);

  SerialBT.println("Ganhos carregados da EEPROM.");
}
