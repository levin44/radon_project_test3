// Modified by: Nisal v0.0.1
// ChtGt 2025-10-11
// This code is used to brute scan all the registers of the RADON sensor
// The sensor is a HS-100C / PM04 Radon Sensor
// The sensor is connected to the ESP32 via I2C
// The sensor is connected to the ESP32 via I2C
#include <Wire.h>
#define SENSOR_ADDR 0x6B

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Scanning registers 0x00–0x3F...");
  for (uint8_t reg = 0x00; reg <= 0x3F; reg++) {
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(SENSOR_ADDR, 2);
    if (Wire.available() >= 2) {
      uint8_t b1 = Wire.read();
      uint8_t b2 = Wire.read();
      Serial.printf("Reg 0x%02X: 0x%02X 0x%02X\n", reg, b1, b2);
    }
    delay(10);
  }
}

void loop(){}
