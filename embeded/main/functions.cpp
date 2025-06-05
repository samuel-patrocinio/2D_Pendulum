#include "functions.h"
#include <Wire.h>
#include <FastLED.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / M_PI)

float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float alpha = 0.7;
int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float robot_angleX, robot_angleY;
float roll_m1, roll_m2;
float gyro_m1, gyro_m2;
float Acc_angleX, Acc_angleY;
int loop_time = 15;
float Gyro_amount = 0.996;
int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int16_t  GyX_offset = 0;
volatile long enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int state1 = 0, state2 = 0, state3 = 0;
bool is_vertical = false;
bool is_running = false;
OffsetsObj offsets;
int16_t motor1_speed = 0;         
int16_t motor2_speed = 0;         
int16_t motor3_speed = 0;
BluetoothSerial SerialBT;

Matrix Rz = {{
    {-0.5000, -0.8660, 0.0000},
    { 0.8660, -0.5000, 0.0000},
    { 0.0000,  0.0000, 1.0000}
}};

//Inverse of Rz(x) = Rz(-x) 
Matrix Rz_inv = {{
    {-0.5000,  0.8660, 0.0000},
    {-0.8660, -0.5000, 0.0000},
    { 0.0000,  0.0000, 1.0000}
}};

void readGyroscope() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43); // Registro inicial do giroscópio
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050, (int)6, (int)true);
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void readAccelerometer() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B); // Registro inicial do acelerômetro
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050, (int)6, (int)true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
}

void applyOffsets() {
  AcXc = AcX - offsets.acX;
  AcYc = AcY - offsets.acY;
  AcZc = AcZ - offsets.acZ + 16384;

  GyX -= GyX_offset;
  GyY -= GyY_offset;
  GyZ -= GyZ_offset;
}

Matrix matmul(Matrix A, Matrix B) {
    Matrix C;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            C.m[i][j] = 0;
            for (int k = 0; k < 3; k++)
                C.m[i][j] += A.m[i][k] * B.m[k][j];
        }
    return C;
}

Matrix euler_to_rotation_matrix(float roll_deg, float pitch_deg, float yaw_deg) {
    float roll  = DEG2RAD(roll_deg);   // X
    float pitch = DEG2RAD(pitch_deg);  // Y
    float yaw   = DEG2RAD(yaw_deg);    // Z

    Matrix R;

    double cx = cosf(roll);
    double sx = sinf(roll);
    double cy = cosf(pitch);
    double sy = sinf(pitch);
    double cz = cosf(yaw);
    double sz = sinf(yaw);

    // Multiplicação R = Rz * Ry * Rx
    R.m[0][0] = cz * cy;
    R.m[0][1] = cz * sy * sx - sz * cx;
    R.m[0][2] = cz * sy * cx + sz * sx;

    R.m[1][0] = sz * cy;
    R.m[1][1] = sz * sy * sx + cz * cx;
    R.m[1][2] = sz * sy * cx - cz * sx;

    R.m[2][0] = -sy;
    R.m[2][1] = cy * sx;
    R.m[2][2] = cy * cx;

    return R;
}

void printMatrix(Matrix mat) {
    for (int i = 0; i < 3; i++) {
        SerialBT.print("[ ");
        for (int j = 0; j < 3; j++) {
            SerialBT.print(mat.m[i][j]);SerialBT.print(" ");
        }
        SerialBT.println("]");
    }
}

void rotation_matrix_to_euler_zyx(Matrix R, float* roll, float* pitch, float* yaw) {
    if (R.m[2][0] < 1) {
        if (R.m[2][0] > -1) {
            *pitch = asinf(-R.m[2][0]);
            *roll  = atan2f(R.m[2][1], R.m[2][2]);
            *yaw   = atan2f(R.m[1][0], R.m[0][0]);
        } else {
            // R.m[2][0] == -1
            *pitch = M_PI / 2;
            *roll  = -atan2f(-R.m[1][2], R.m[1][1]);
            *yaw   = 0;
        }
    } else {
        // R.m[2][0] == 1
        *pitch = -M_PI / 2;
        *roll  = -atan2f(-R.m[1][2], R.m[1][1]);
        *yaw   = 0;
    }
}

void ang_vel_calc(){

  gyroX = GyX / 131.0;
  gyroY = GyY / 131.0;
  gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
  gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;

  //VELOCIDADE ANGULAR EM M1
  gyro_m1 = -gyroXfilt * 0.5 + gyroYfilt * 0.8660;
  gyro_m2 = -gyroXfilt * 0.8660 - gyroYfilt * 0.5;

};

void angle_calc() {
  readGyroscope();
  readAccelerometer();
  applyOffsets();

  float gyro_factor = loop_time / 1000.0 / 65.536;

  // Cálculo do ângulo em Y com filtro complementar
  robot_angleY += GyY * gyro_factor;
  Acc_angleY = -atan2(AcXc, AcZc) * 57.2958;
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  // Cálculo do ângulo em X com filtro complementar
  robot_angleX += GyX * gyro_factor;
  Acc_angleX = atan2(AcYc, AcZc) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  // ANGULOS EM M1
  Matrix R = euler_to_rotation_matrix(robot_angleX, robot_angleY, 0);
  Matrix temp = matmul(R,Rz);
  Matrix m1 = matmul(Rz_inv,temp);
  float roll_rad_m1, pitch_rad_m1, yaw_rad_m1;
  rotation_matrix_to_euler_zyx(m1, &roll_rad_m1, &pitch_rad_m1, &yaw_rad_m1);
  roll_m1 = RAD2DEG(roll_rad_m1);

  // ANGULOS EM M2
  temp = matmul(R,Rz_inv);
  Matrix m2 = matmul(Rz,temp);
  float roll_rad_m2, pitch_rad_m2, yaw_rad_m2;
  rotation_matrix_to_euler_zyx(m2, &roll_rad_m2, &pitch_rad_m2, &yaw_rad_m2);
  roll_m2 = RAD2DEG(roll_rad_m2);

  // Verificação de verticalidade
  is_vertical = abs(robot_angleX) < 20.0 && abs(robot_angleY) < 20.0;

  if (!is_vertical) {
    is_running = false;
  }
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay(100);

  writeTo(MPU6050, PWR_MGMT_1, 0);                    // Acorda o sensor
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);       // Configura sensibilidade do acelerômetro
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);       // Configura sensibilidade do giroscópio
  delay(100);

  GyX_offset = calibrateGyroAxis('X');
  GyY_offset = calibrateGyroAxis('Y');
  GyZ_offset = calibrateGyroAxis('Z');
}

int calibrateGyroAxis(char axis) {
  long sum = 0;

  for (int i = 0; i < 512; i++) {
    angle_calc();  // Popula GyX, GyY, GyZ
    switch (axis) {
      case 'X': sum += GyX; break;
      case 'Y': sum += GyY; break;
      case 'Z': sum += GyZ; break;
    }
    delay(5);
  }

  int offset = sum >> 9; // Equivale a dividir por 512
  SerialBT.print("Gy"); SerialBT.print(axis); SerialBT.print(" offset = ");
  SerialBT.println(offset);
  return offset;
}

int Tuning() {
  if (is_running) return 0;

  // Verifica giro horário (≥ 90° → 400 pulsos)
  if (enc_count1 >= 400) {

    offsets.acX = AcX;
    offsets.acY = AcY;
    offsets.acZ = AcZ;
    offsets.ID = 96;

    EEPROM.put(0, offsets);

    enc_count1 = 0;
    is_running = true;
    return 1;
  }

  if (enc_count1 <= -100) {
    EEPROM.get(0, offsets);
    enc_count1 = 0;
    is_running = true;
    return 1;
  }

  return 0;
}



void encoderRead(int pinA, int pinB, volatile long &count, int &state) {
  state = ((state << 2) | (digitalRead(pinA) << 1) | digitalRead(pinB)) & 0x0F;

  if (state == 0x02 || state == 0x0D || state == 0x04 || state == 0x0B) {
    count++;
  } else if (state == 0x01 || state == 0x0E || state == 0x08 || state == 0x07) {
    count--;
  }
}

void ENC1_READ() {
  encoderRead(ENC1_1, ENC1_2, enc_count1, state1);
}

void ENC2_READ() {
  encoderRead(ENC2_1, ENC2_2, enc_count2, state2);
}

void ENC3_READ() {
  encoderRead(ENC3_1, ENC3_2, enc_count3, state3);
}

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor1_control(int sp) {
  sp = sp + motor1_speed;
  if (sp < 0)
    digitalWrite(DIR1, LOW);
  else
    digitalWrite(DIR1, HIGH);
  pwmSet(PWM1_CH, 255 - abs(sp));
}
 
void Motor2_control(int sp) {
  sp = sp + motor2_speed;
  if (sp < 0)
    digitalWrite(DIR2, LOW);
  else
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2_CH, 255 - abs(sp));
}
 
void Motor3_control(int sp) {
  sp = sp + motor3_speed;
  if (sp < 0)
    digitalWrite(DIR3, LOW);
  else
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3_CH, 255 - abs(sp));
}

void setupMotor(int encA, int encB, void (*isr)(), int brakePin, int dirPin, int pwmPin, int pwmChannel) {
  // Encoder pins
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);

  // Interrupts to read encoder
  attachInterrupt(encA, isr, CHANGE);
  attachInterrupt(encB, isr, CHANGE);

  // Motor direction and brake
  pinMode(brakePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(brakePin, HIGH);

  // PWM setup
  ledcSetup(pwmChannel, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(pwmPin, pwmChannel);
}

void setGains(float *k1, float *k2, float *k3, float *k4) {

  if (is_running) return;

  String input = SerialBT.readStringUntil('\n');
  input.trim();

  if (input == "print") {
    SerialBT.printf("k1=%.2f k2=%.2f k3=%.2f k4=%.3f\n", *k1, *k2, *k3, *k4);
    return;
  }

  if (input.startsWith("k1=")) {
    *k1 = input.substring(3).toFloat();
    SerialBT.print("k1 atualizado para: "); SerialBT.println(*k1);
  } else if (input.startsWith("k2=")) {
    *k2 = input.substring(3).toFloat();
    SerialBT.print("k2 atualizado para: "); SerialBT.println(*k2);
  } else if (input.startsWith("k3=")) {
    *k3 = input.substring(3).toFloat();
    SerialBT.print("k3 atualizado para: "); SerialBT.println(*k3);
  } else if (input.startsWith("k4=")) {
    *k4 = input.substring(3).toFloat();
    SerialBT.print("k4 atualizado para: "); SerialBT.println(*k4);
  } else {
    return;
  }
}





