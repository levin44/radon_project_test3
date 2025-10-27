
// Serial Tap for ESP32 (Arduino)
// Listens on two UARTs (receive-only) and prints to USB Serial console.
// WARNING: DO NOT connect ±12V RS232 directly to ESP32. Use level shifter (MAX232) or TTL level shifter.

#include <Arduino.h>

// Pins you choose for RX taps (change as needed)
const int RX_PIN_SIDE_A = 16; // e.g. device -> PC TX tap
const int RX_PIN_SIDE_B = 4;  // e.g. PC -> device TX tap

// Which UART numbers to use (ESP32 supports UART 0,1,2)
HardwareSerial TapA(1); // UART1
HardwareSerial TapB(2); // UART2

// Baud configuration
bool autoScanBaud = true;   // try multiple common baud rates if true
int fixedBaud = 9600;       // used if autoScanBaud == false
int uartConfig = SERIAL_8N1; // change if you need parity/stopbits (SERIAL_7E1, etc)

// Bauds to try (if auto scanning)
const int baudList[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200};
const int NUM_BAUDS = sizeof(baudList)/sizeof(baudList[0]);

unsigned long lastPrint = 0;

String hexPrintable(uint8_t b) {
  if (b >= 32 && b <= 126) {
    // printable ascii
    return String((char)b);
  } else {
    char buf[8];
    sprintf(buf, "\\x%02X", b);
    return String(buf);
  }
}

void printFromTap(HardwareSerial &ser, const char *label) {
  while (ser.available()) {
    int c = ser.read();
    // print timestamp, label, ascii (or hex if non-printable)
    if (c >= 32 && c <= 126) {
      Serial.printf("%010lu [%s] %c\n", millis(), label, (char)c);
    } else {
      Serial.printf("%010lu [%s] %s\n", millis(), label, hexPrintable((uint8_t)c).c_str());
    }
  }
}

void beginTap(int baud) {
  // beginTap on both UARTs with RX pin only (txPin = -1)
  // Serial.begin is USB console
  TapA.begin(baud, uartConfig, RX_PIN_SIDE_A, -1);
  TapB.begin(baud, uartConfig, RX_PIN_SIDE_B, -1);
  Serial.printf("Tapping started at %d baud (RX pins A=%d, B=%d)\n", baud, RX_PIN_SIDE_A, RX_PIN_SIDE_B);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(5); } // wait for USB serial
  Serial.println();
  Serial.println("ESP32 Serial Tap - starting...");

  if (!autoScanBaud) {
    beginTap(fixedBaud);
  } else {
    // quick brute-force: try each baud for a short time and print what we see
    for (int i = 0; i < NUM_BAUDS; ++i) {
      int b = baudList[i];
      Serial.printf("Trying baud %d for 300 ms...\n", b);
      TapA.begin(b, uartConfig, RX_PIN_SIDE_A, -1);
      TapB.begin(b, uartConfig, RX_PIN_SIDE_B, -1);

      unsigned long t0 = millis();
      while (millis() - t0 < 300) {
        printFromTap(TapA, "A");
        printFromTap(TapB, "B");
        delay(5);
      }
      // stop to try next rate
      TapA.end();
      TapB.end();
    }
    // After scan, start on the most common (115200) — change if you prefer
    beginTap(115200);
  }
}

void loop() {
  // read and print any data from the tapped lines
  printFromTap(TapA, "A"); // label "A" = RX_PIN_SIDE_A (device->PC)
  printFromTap(TapB, "B"); // label "B" = RX_PIN_SIDE_B (PC->device)

  // small delay to avoid hogging CPU
  delay(1);
}