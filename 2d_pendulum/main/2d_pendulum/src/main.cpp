#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
 
#include <FastLED.h>
 
BluetoothSerial SerialBT;

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}
 
void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit();
    EEPROM.get(0, offsets);
    if (offsets.ID == 96) calibrated = true;
    calibrating = false;
    SerialBT.println("Calibrating off.");
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050, (int)6, (int)true);

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
 
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050, (int)6, (int)true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = -Wire.read() << 8 | Wire.read();
  AcZ = -Wire.read() << 8 | Wire.read();
 
  if (abs(AcX) < 2000) {
    AcXc = AcX - offsets.acXv;
    AcYc = AcY - offsets.acYv;
    AcZc = AcZ - offsets.acZv;
  } else {
    AcXc = AcX - offsets.acXe;
    AcYc = AcY - offsets.acYe;
    AcZc = AcZ - offsets.acZe;
  }
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;
 
  robot_angleY += GyY * loop_time / 1000 / 65.536;
  Acc_angleY = atan2(AcXc, -AcZc) * 57.2958;
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
 
  robot_angleX += GyX * loop_time / 1000 / 65.536;
  Acc_angleX = -atan2(AcYc, -AcZc) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);
 
  if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_vertex = true;
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_edge = true;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_vertex) {
    vertical_vertex = false;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_edge) {
    vertical_edge = false;
  }
}
 
void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
  
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
 
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
 
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
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
 
void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  int16_t m1 = round((0.5 * pwm_X - 0.866 * pwm_Y) / 1.37 + pwm_Z);  
  int16_t m2 = round((0.5 * pwm_X + 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m3 = pwm_X;  
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}
 
void threeWay_to_XY(int m1_speed, int m2_speed, int m3_speed) {
  speed_X = (m1_speed - m2_speed) * 0.866;
  speed_Y = (m1_speed + m2_speed) * 0.5 - m3_speed;
}
 
void ENC1_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}
 
void ENC2_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}
 
void ENC3_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}
 
int Tuning() {
  if (!SerialBT.available()) return 0;
 
  String input = SerialBT.readStringUntil('\n');
  input.trim();
 
  // Comando de calibração (mantido do código original)
  if (input == "c+") {
    if (!calibrating) {
      calibrating = true;
      SerialBT.println("🔧 Calibração iniciada. Coloque o cubo no vértice...");
    }
    return 1;
  }
 
  if (input == "c-") {
    SerialBT.print("X: "); SerialBT.print(AcX); SerialBT.print(" Y: "); SerialBT.print(AcY); SerialBT.print(" Z: "); SerialBT.println(AcZ + 16384);
    if (abs(AcX) < 2000 && abs(AcY) < 2000) {
      offsets.ID = 96;
      offsets.acXv = AcX;
      offsets.acYv = AcY;
      offsets.acZv = AcZ + 16384;
      SerialBT.println("✅ Vértice calibrado. Agora coloque na aresta.");
      save();
      vertex_calibrated = true;
 
    } //else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated) {
      //offsets.acXe = AcX;
      //offsets.acYe = AcY;
      //offsets.acZe = AcZ + 16384;
      //SerialBT.println("✅ Aresta calibrada.");
      //save();
    //} 
    else {
      SerialBT.println("⚠️ Ângulos inválidos para calibração.");
    }
    return 1;
  }
 
  // Comando de exibição dos valores atuais
  if (input == "print") {
    SerialBT.printf("K1=%.2f K2=%.2f K3=%.2f K4=%.3f\n", K1, K2, K3, K4);
    SerialBT.printf("zK2=%.2f zK3=%.2f\n", zK2, zK3);
    SerialBT.printf("eK1=%.2f eK2=%.2f eK3=%.2f eK4=%.3f\n", eK1, eK2, eK3, eK4);
    return 1;
  }
 
if (input.startsWith("K1=")) {
    K1 = input.substring(3).toFloat();
    SerialBT.print("K1 atualizado para: "); SerialBT.println(K1);
    return 1;
  } else if (input.startsWith("K2=")) {
    K2 = input.substring(3).toFloat();
    SerialBT.print("K2 atualizado para: "); SerialBT.println(K2);
    return 1;
  } else if (input.startsWith("K3=")) {
    K3 = input.substring(3).toFloat();
    SerialBT.print("K3 atualizado para: "); SerialBT.println(K3);
    return 1;
  } else if (input.startsWith("K4=")) {
    K4 = input.substring(3).toFloat();
    SerialBT.print("K4 atualizado para: "); SerialBT.println(K4);
    return 1;
  } else if (input.startsWith("zK2=")) {
    zK2 = input.substring(4).toFloat();
    SerialBT.print("zK2 atualizado para: "); SerialBT.println(zK2);
    return 1;
  } else if (input.startsWith("zK3=")) {
    zK3 = input.substring(4).toFloat();
    SerialBT.print("zK3 atualizado para: "); SerialBT.println(zK3);
    return 1;
  } else if (input.startsWith("eK1=")) {
    eK1 = input.substring(4).toFloat();
    SerialBT.print("eK1 atualizado para: "); SerialBT.println(eK1);
    return 1;
  } else if (input.startsWith("eK2=")) {
    eK2 = input.substring(4).toFloat();
    SerialBT.print("eK2 atualizado para: "); SerialBT.println(eK2);
    return 1;
  } else if (input.startsWith("eK3=")) {
    eK3 = input.substring(4).toFloat();
    SerialBT.print("eK3 atualizado para: "); SerialBT.println(eK3);
    return 1;
  } else if (input.startsWith("eK4=")) {
    eK4 = input.substring(4).toFloat();
    SerialBT.print("eK4 atualizado para: "); SerialBT.println(eK4);
    return 1;
  } else {
    SerialBT.println("Comando invalido! Use formato GANHO=VALOR. Exemplo: K1=180");
    return 1;
  }
}
 
void releaseBrakes() {
  digitalWrite(BRAKE1, LOW);
  digitalWrite(BRAKE2, LOW);
  digitalWrite(BRAKE3, LOW);
}
 
void applyBrakes() {
  digitalWrite(BRAKE1, HIGH);
  digitalWrite(BRAKE2, HIGH);
  digitalWrite(BRAKE3, HIGH);
}
 
void setup() {
  Serial.begin(115200);
  SerialBT.begin("2d-pendulum"); // Bluetooth device name
 
  EEPROM.begin(EEPROM_SIZE);
 
  pinMode(BRAKE1, OUTPUT);
  pinMode(BRAKE2, OUTPUT);
  pinMode(BRAKE3, OUTPUT);
 
  digitalWrite(BRAKE1, HIGH);
  digitalWrite(BRAKE2, HIGH);
  digitalWrite(BRAKE3, HIGH);
  
  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);
 
  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
    calibrated = true;
 
  delay(200);
  angle_setup();
}
 
void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {

    Tuning();
    angle_calc();
 
    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;

    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
    
    if (vertical_vertex && calibrated && !calibrating) {    
      applyBrakes();
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      
      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);
 
      motors_speed_X += speed_X / 5;
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(pwm_X, pwm_Y, pwm_Z);

      // pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);
      
      // motors_speed_X += motor3_speed / 5;
      // Motor3_control(pwm_X);
 
 
    } else if (vertical_edge && calibrated && !calibrating) {
      digitalWrite(BRAKE3, HIGH);
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);
      
      motors_speed_X += motor3_speed / 5;
      Motor3_control(pwm_X);
 
      
    } else {
      XYZ_to_threeWay(0, 0, 0);
      releaseBrakes();
      motors_speed_X = 0;
      motors_speed_Y = 0;
    }
    previousT_1 = currentT;

    // Serial.print("robot_angleX: "); Serial.print(robot_angleX);
    // Serial.print(" gyroXfilt: "); Serial.print(gyroXfilt);
    // Serial.print(" speed_Y: "); Serial.print(speed_Y);
    // Serial.print(" motors_speed_Y: "); Serial.print(motors_speed_Y);
    // Serial.print(" motor3_speed: "); Serial.print(motor3_speed);
    // Serial.print(" pwm X: "); Serial.print(pwm_X);
    // Serial.print(" pwm X test: "); Serial.println(pwm_X_test);
  }
  
  if (currentT - previousT_2 >= 2000) {    
    if (!calibrated && !calibrating) {
      SerialBT.println("first you need to calibrate the balancing points...");
      Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
    }
    previousT_2 = currentT;
  }  
}
 