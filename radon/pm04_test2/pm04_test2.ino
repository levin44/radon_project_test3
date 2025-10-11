// Modified by: Nisal v0.0.1
// clode 2025-10-7
// This code is used to read the data from the RADON sensor
// The sensor is a HS-100C / PM04 Radon Sensor
// The sensor is connected to the ESP32 via I2C
// The sensor is connected to the ESP32 via I2C
#include <Wire.h>
byte ADDRESS_SLAVE = 0X6B;
//byte ADDRESS_SLAVE = 0XD7;
byte REGISTER_XY = 0X19;
byte REGISTER_ZZ = 0X19;
byte READ_LENGTH = 1;

void setup() {
Serial.begin (9600);
pinMode (41, OUTPUT) ;
digitalWrite(41, LOW);
delay (50) ;
digitalWrite (41, HIGH);

Wire.begin(0x6B);
//Wire.setClock(400000); // set I2C 'full-speed'
// Wire.beginTransmission (ADDRESS_SLAVE);
// Wire.write(REGISTER_XY); // set register for read
//Wire.endTransmission();
}

void loop() {
    Wire.beginTransmission (ADDRESS_SLAVE);
    Wire.write(REGISTER_XY); // set register for read
    Wire.endTransmission(false);
    Wire.requestFrom(ADDRESS_SLAVE, READ_LENGTH);
    //Wire.write(byte(0X09));
    //byte buff [READ_LENGTH];
    //Wire.readBytes (buff, READ_LENGTH);
    //for (int i = 0; i < READ_LENGTH; i++) {
    // Serial.printin(buff[i], HEX);
    //}

    //Wire.endTransmission();
    while (Wire. available ()) {
        byte c = Wire.read();
        //byte d = (c >> 4); // Read seconds register (0X08)

        //byte e = c & OXOF;
        //Serial.println(d*10+e);

        Serial.println(c);
    }
}