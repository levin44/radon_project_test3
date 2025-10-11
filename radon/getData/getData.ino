#include <Wire.h>

#define RADON_ADDR 0x6B   // I2C address from datasheet (0xD6 write / 0xD7 read)

// Register map (from datasheet)
#define REG_BOOT_PROGRESS  0x19
#define REG_STATE          0x1A
#define REG_PCI_L          0x0D
#define REG_PCI_HL         0x0E
#define REG_PCI_HH         0x0F
#define REG_BQ_L           0x10
#define REG_BQ_HL          0x11
#define REG_BQ_HH          0x12

// Function to read a single byte
uint8_t readByte(uint8_t reg) {
  Wire.beginTransmission(RADON_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(RADON_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// Function to read multiple bytes
void readBytes(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(RADON_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(RADON_ADDR, count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA/SCL default pins for ESP32
  Serial.println("HS-100C / PM04 Radon Sensor Test");

  // Wait for sensor boot to complete (6 minutes typical)
  Serial.println("Waiting for sensor boot (up to 6 min)...");
  while (true) {
    uint8_t boot = readByte(REG_BOOT_PROGRESS);
    uint8_t state = readByte(REG_STATE);
    Serial.printf("Boot progress: %d%%, State: 0x%02X\n", boot, state);
    if (boot >= 100 && ((state >> 6) & 0x03) == 0) { // bit7-6 == 0 → normal
      Serial.println("Sensor ready!");
      break;
    }
    delay(5000); // check every 5 seconds
  }
}

void loop() {
  uint8_t pciBytes[3];
  uint8_t bqBytes[3];

  // Read pCi/L registers (0x0D–0x0F)
  readBytes(REG_PCI_L, 3, pciBytes);
  // Read Bq/m³ registers (0x10–0x12)
  readBytes(REG_BQ_L, 3, bqBytes);

  // According to datasheet:
  // Example pCi 123.45 = [L=45, HL=123, HH=0]
  float radon_pci = pciBytes[1] + (pciBytes[0] / 100.0f);
  float radon_bq  = bqBytes[1] + (bqBytes[0] / 100.0f);

  Serial.println("------ RADON DATA ------");
  Serial.printf("pCi/L  = %.2f\n", radon_pci);
  Serial.printf("Bq/m³  = %.2f\n", radon_bq);

  // Optional: print raw register bytes
  Serial.printf("Raw pCi bytes: %02X %02X %02X\n", pciBytes[0], pciBytes[1], pciBytes[2]);
  Serial.printf("Raw Bq  bytes: %02X %02X %02X\n", bqBytes[0], bqBytes[1], bqBytes[2]);
  Serial.println("------------------------\n");

  delay(10000);  // read every 10 seconds
}
