// ESP32: RD200M Test with HardwareSerial
// Connect RD200M (or Arduino emulator) TX -> ESP32 RX (GPIO16 here)
// Connect RD200M RX (if needed) -> ESP32 TX (GPIO17 here)

#define RXD2 16
#define TXD2 17

void setup() {
  // Debug serial to PC
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Serial.println("ESP32 RD200M Test Starting...");

  // Hardware Serial2 for sensor/emulator
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial2 ready (listening on pins RX=16, TX=17)");
}

void loop() {
  // Check if emulator/sensor sent data
  while (Serial2.available()) {
    byte b = Serial2.read();
    Serial.print("0x");
    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }
  delay(100); // small pause
}
