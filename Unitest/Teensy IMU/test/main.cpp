#include <Arduino.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68
#define MAG_ADDR     0x0C

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 1);
  return Wire.read();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, count);
  for (int i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}

void setupMPU() {
  writeRegister(0x6B, 0x00); // Wake up from sleep
  delay(100);
  writeRegister(0x1A, 0x03); // DLPF config
  writeRegister(0x1B, 0x00); // Gyro full scale ±250dps
  writeRegister(0x1C, 0x00); // Accel full scale ±2g
  writeRegister(0x37, 0x02); // Enable bypass to magnetometer
}

void readAccelGyro(int16_t* accel, int16_t* gyro) {
  uint8_t raw[14];
  readRegisters(0x3B, 14, raw);

  accel[0] = (raw[0] << 8) | raw[1];
  accel[1] = (raw[2] << 8) | raw[3];
  accel[2] = (raw[4] << 8) | raw[5];

  gyro[0] = (raw[8] << 8) | raw[9];
  gyro[1] = (raw[10] << 8) | raw[11];
  gyro[2] = (raw[12] << 8) | raw[13];
}

void setupMag() {
  // Power down magnetometer
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  // Enter fuse ROM mode
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x0F);
  Wire.endTransmission();
  delay(10);

  // Leave fuse mode, set to 16-bit continuous measurement mode 100Hz
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x16);
  Wire.endTransmission();
}

void readMag(int16_t* mag) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03); // Start reading at HXL
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 7);

  if (Wire.available() == 7) {
    uint8_t raw[7];
    for (int i = 0; i < 7; i++) raw[i] = Wire.read();

    mag[0] = (raw[1] << 8) | raw[0]; // X
    mag[1] = (raw[3] << 8) | raw[2]; // Y
    mag[2] = (raw[5] << 8) | raw[4]; // Z
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupMPU();
  setupMag();

  delay(100);
  Serial.println("MPU9250 initialized.");
}

void loop() {
  int16_t accel[3], gyro[3], mag[3];

  readAccelGyro(accel, gyro);
  readMag(mag);

  Serial.print(accel[0]); Serial.print(";");
  Serial.print(accel[1]); Serial.print(";");
  Serial.print(accel[2]); Serial.print(";");
  Serial.print(gyro[0]); Serial.print(";");
  Serial.print(gyro[1]); Serial.print(";");
  Serial.print(gyro[2]); Serial.print(";");
  Serial.print(mag[0]); Serial.print(";");
  Serial.print(mag[1]); Serial.print(";");
  Serial.println(mag[2]);

  delay(10);
}
