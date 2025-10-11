// Modified by: Nisal v0.0.1
// ChtGt 2025-10-11
// This code is used to verify the device response
// got 21:24:33.196 -> Reading register 0x00... 21:24:33.196 -> 0x0 0x0
//  ESP32 successfully talked to the device at address 0x6B
// The device acknowledged the transmission and returned data (0x00 0x00)
// But it returned zeros → which means your read sequence is syntactically valid but not what the device expects to deliver real data.
// The sensor is a HS-100C / PM04 Radon Sensor
// The sensor is connected to the ESP32 via I2C
// The sensor is connected to the ESP32 via I2C
#include <Wire.h>
#define SENSOR_ADDR 0x6B

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Reading register 0x00...");
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, 2);
  while(Wire.available()){
    byte b = Wire.read();
    Serial.print("0x"); Serial.print(b, HEX); Serial.print(" ");
  }
  Serial.println();
}

void loop(){}
