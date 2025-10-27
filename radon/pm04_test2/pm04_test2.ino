// Modified by: Nisal v0.0.2
// code 2025-10-7
// This code is used to read the data from the RADON sensor
// The sensor is a HS-100C / PM04 Radon Sensor
// The sensor is connected to the ESP32 via I2C
// ESP32 + PM04 (HS-100 C) Radon Sensor via I2C
// - Reads "Average of the previous 10 minutes" registers every second
// - Prints pCi/L and Bq/m^3 to Serial Monitor

#include <Wire.h>

// I2C slave 7-bit address for PM04
static const uint8_t PM04_I2C_ADDR = 0x6B; // 0b1101011

// ESP32 default I2C pins (change if wired differently)
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;

// Register map (subset used)
static const uint8_t REG_AVG10_PCI_L  = 0x0D; // fractional (2 decimal digits)
static const uint8_t REG_AVG10_PCI_HL = 0x0E; // integer low byte
static const uint8_t REG_AVG10_PCI_HH = 0x0F; // integer high byte
static const uint8_t REG_AVG10_BQ_L   = 0x10; // fractional (2 decimal digits)
static const uint8_t REG_AVG10_BQ_HL  = 0x11; // integer low byte
static const uint8_t REG_INITIAL_BOOT = 0x19; // 0..100 (%) ~6 minutes
static const uint8_t REG_STATE        = 0x1A; // status packed in nibbles

// Forward declare status struct to satisfy Arduino's auto-generated prototypes
struct Pm04Status;

// Decode packed status fields from REG_STATE (0x1A)
struct Pm04Status {
  uint8_t sensorStatus;   // bits 7..6: 0=Normal, 1=Abnormal, 2=Boot
  uint8_t i2cStatus;      // bits 5..4: 0=Normal, 1=Normal w/correction, 2=I2C Error
  uint8_t vibration;      // bits 3..2: 0=No vibration, 1=Vibration
  uint8_t errorStatus;    // bits 1..0: 01 after power-on until healthy I2C for >3s
};

// Helper to write a register address then read N bytes
static bool i2cRead(uint8_t deviceAddr, uint8_t regAddr, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  uint8_t txStatus = Wire.endTransmission(false); // repeated start
  if (txStatus != 0) {
    Serial.print("I2C TX error: ");
    Serial.println(txStatus);
    return false;
  }

  uint8_t numRead = Wire.requestFrom((int)deviceAddr, (int)length, (int)true);
  if (numRead != length) {
    Serial.print("I2C RX error: expected ");
    Serial.print(length);
    Serial.print(", got ");
    Serial.println(numRead);
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

static bool readStatus(uint8_t &bootPercent, Pm04Status &status) {
  uint8_t v;
  if (!i2cRead(PM04_I2C_ADDR, REG_INITIAL_BOOT, &bootPercent, 1)) return false;
  if (!i2cRead(PM04_I2C_ADDR, REG_STATE, &v, 1)) return false;
  status.sensorStatus = (v >> 6) & 0x03;
  status.i2cStatus    = (v >> 4) & 0x03;
  status.vibration    = (v >> 2) & 0x03;
  status.errorStatus  = (v >> 0) & 0x03;
  return true;
}

// Read previous 10-minute averages for pCi/L and Bq/m^3
// According to datasheet example: pCi 123.45 => L=45, HL=123, HH=0
// So value = (HH<<8 | HL) + (L / 100.0)
static bool readPrev10MinAverages(float &pCiPerL, float &bqPerM3) {
  uint8_t buf[3];
  if (!i2cRead(PM04_I2C_ADDR, REG_AVG10_PCI_L, buf, 3)) return false;
  uint16_t pciInteger = (uint16_t(buf[2]) << 8) | uint16_t(buf[1]);
  uint8_t  pciFraction = buf[0]; // 0..99
  pCiPerL = float(pciInteger) + (float(pciFraction) / 100.0f);

  if (!i2cRead(PM04_I2C_ADDR, REG_AVG10_BQ_L, buf, 3)) return false;
  uint16_t bqInteger = (uint16_t(buf[2]) << 8) | uint16_t(buf[1]);
  uint8_t  bqFraction = buf[0]; // 0..99
  bqPerM3 = float(bqInteger) + (float(bqFraction) / 100.0f);
  return true;
}

// I2C Scanner function to detect devices
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int deviceCount = 0;
  
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("PM04 Radon (prev 10-min average) over I2C @0x6B");
  Serial.println("Initializing I2C bus...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000); // 100kHz is plenty
  delay(100); // Give more time for initialization
  
  // Scan for I2C devices
  scanI2C();
  
  // Test specific PM04 address
  Serial.print("Testing PM04 at address 0x6B...");
  Wire.beginTransmission(PM04_I2C_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    Serial.println(" OK - Device responding");
  } else {
    Serial.print(" FAILED - Error code: ");
    Serial.println(error);
    Serial.println("Possible issues:");
    Serial.println("- Check wiring (SDA=21, SCL=22)");
    Serial.println("- Verify 12V power supply");
    Serial.println("- Check sensor is properly connected");
  }
  Serial.println();
}

void loop() {
  static unsigned long lastScanTime = 0;
  static int consecutiveErrors = 0;
  
  uint8_t boot = 0;
  Pm04Status st{};
  
  // Try reading status with retry
  bool statusRead = false;
  for (int retry = 0; retry < 3; retry++) {
    if (readStatus(boot, st)) {
      statusRead = true;
      consecutiveErrors = 0;
      break;
    }
    delay(100); // Short delay between retries
  }
  
  if (!statusRead) {
    consecutiveErrors++;
    Serial.print("I2C read error (status) - Attempt ");
    Serial.print(consecutiveErrors);
    Serial.println(". Check wiring and power (12V).");
    
    // Rescan I2C bus every 10 errors
    if (consecutiveErrors % 10 == 0) {
      Serial.println("Rescanning I2C bus...");
      scanI2C();
    }
    
    delay(2000); // Longer delay on error
    return;
  }

  // During boot, sensor reports boot% and does not count measurement time
  if (boot < 100 || st.sensorStatus == 2) {
    Serial.print("Booting: ");
    Serial.print(boot);
    Serial.print("%  ");
    Serial.print("I2C:"); Serial.print(st.i2cStatus);
    Serial.print("  Err:"); Serial.print(st.errorStatus);
    Serial.print("  Vib:"); Serial.println(st.vibration);
    delay(1000);
    return;
  }

  if (st.i2cStatus == 2) {
    Serial.println("Warning: PM04 reports I2C Error status.");
  }
  if (st.vibration != 0) {
    Serial.println("Notice: Vibration detected – counts are paused.");
  }

  float pCi = 0.0f;
  float bq  = 0.0f;
  
  // Try reading data with retry
  bool dataRead = false;
  for (int retry = 0; retry < 3; retry++) {
    if (readPrev10MinAverages(pCi, bq)) {
      dataRead = true;
      break;
    }
    delay(100); // Short delay between retries
  }
  
  if (!dataRead) {
    Serial.println("I2C read error (avg data) - retrying...");
    delay(1000);
    return;
  }

  Serial.print("Prev10min pCi/L: ");
  Serial.print(pCi, 2);
  Serial.print("  |  Bq/m3: ");
  Serial.print(bq, 2);
  Serial.print("  |  Vib:");
  Serial.print(st.vibration);
  Serial.print("  I2C:");
  Serial.print(st.i2cStatus);
  Serial.print("  Err:");
  Serial.println(st.errorStatus);

  // Values update every 10 minutes; print once per second for visibility
  delay(1000);
}