#include <Wire.h>
void setup(){
  Serial.begin(115200);
  Wire.begin(); // default SDA, SCL pins or Wire.begin(SDA_pin, SCL_pin);
  delay(1000);
  Serial.println("\nI2C Scanner");
  for (byte addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0){
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    } else if (err == 4) {
      Serial.print("Unknown error at 0x");
      Serial.println(addr, HEX);
    }
    delay(5);
  }
  Serial.println("Done");
}
void loop(){}
