/*
  Sleep Guardian — Arduino/ESP32 Sensor Reader

  Wiring:
    MPU6050 (accelerometer):  SDA → A4, SCL → A5 (or D21/D22 on ESP32)
    FSR (pressure sensor):    One leg → 5V, other leg → A0 + 10kΩ resistor to GND
    Sound sensor (analog):    OUT → A1, VCC → 5V, GND → GND

  Sends JSON over serial at 9600 baud, once per second:
    {"sound":52.3,"pressure":612.0,"position":"back","accel_magnitude":1.45}
*/

#include <Wire.h>

// ---- MPU6050 registers ----
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ---- Pin definitions ----
#define FSR_PIN A0
#define SOUND_PIN A1

// ---- Position thresholds (tune for your pillow mounting) ----
// Based on accelerometer X, Y, Z orientation when MPU6050 is on the pillow
// These values assume MPU6050 mounted flat on pillow surface
#define BACK_Z_MIN 0.7
#define SIDE_X_THRESHOLD 0.5
#define STOMACH_Z_MAX -0.5

float accelX, accelY, accelZ;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(100);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // Convert to g (±2g range, 16384 LSB/g)
  accelX = rawX / 16384.0;
  accelY = rawY / 16384.0;
  accelZ = rawZ / 16384.0;
}

String classifyPosition() {
  // Determine sleep position from accelerometer orientation
  if (accelZ > BACK_Z_MIN) {
    return "back";
  } else if (accelZ < STOMACH_Z_MAX) {
    return "stomach";
  } else if (accelX > SIDE_X_THRESHOLD) {
    return "side_left";
  } else if (accelX < -SIDE_X_THRESHOLD) {
    return "side_right";
  }
  return "back";
}

void loop() {
  // 1. Read MPU6050 accelerometer
  readMPU6050();
  float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  String position = classifyPosition();

  // 2. Read FSR pressure sensor (0-1023 → 0-1023 raw)
  int fsrRaw = analogRead(FSR_PIN);
  float pressure = (float)fsrRaw;

  // 3. Read sound sensor (analog, 0-1023 → map to ~20-100 dB range)
  int soundRaw = analogRead(SOUND_PIN);
  float sound = map(soundRaw, 0, 1023, 20, 100);

  // 4. Send JSON over serial
  Serial.print("{\"sound\":");
  Serial.print(sound, 1);
  Serial.print(",\"pressure\":");
  Serial.print(pressure, 1);
  Serial.print(",\"position\":\"");
  Serial.print(position);
  Serial.print("\",\"accel_magnitude\":");
  Serial.print(magnitude, 2);
  Serial.println("}");

  delay(1000);  // 1 reading per second
}
