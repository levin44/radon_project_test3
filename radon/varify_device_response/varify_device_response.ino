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
