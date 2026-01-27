#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <time.h>

// -------- WIFI --------
const char* ssid = "Nebula";
const char* password = "2024niDI";

// -------- NTP --------
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;        // change if needed
const int   daylightOffset_sec = 0;

// -------- SERVER --------
WebServer server(80);

// -------- LOGGING --------
const char* filename = "/data.csv";
unsigned long lastLogTime = 0;
const unsigned long logInterval = 2000; // 10 minutes (ms)

// -------- FAKE SENSOR --------
float readSensor() {
  return 23.4;  // replace with real sensor later
}

// -------- APPEND CSV --------
void appendCSV(time_t timestamp, float value) {
  File file = LittleFS.open(filename, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }

  file.printf("%ld,%.2f\n", timestamp, value);
  file.close();
}

// -------- HTTP HANDLERS --------
void handleDownload() {
  if (!LittleFS.exists(filename)) {
    server.send(404, "text/plain", "No data file");
    return;
  }

  File file = LittleFS.open(filename, FILE_READ);
  server.streamFile(file, "text/csv");
  file.close();
}

void handleClear() {
  LittleFS.remove(filename);
  server.send(200, "text/plain", "Data cleared");
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    return;
  }

  // Create CSV header if file doesn't exist
  if (!LittleFS.exists(filename)) {
    File file = LittleFS.open(filename, FILE_WRITE);
    file.println("timestamp,value");
    file.close();
  }

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Waiting for NTP time...");
  time_t now;
  while ((now = time(nullptr)) < 100000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nTime synced");

  // HTTP server
  server.on("/data.csv", HTTP_GET, handleDownload);
  server.on("/clear", HTTP_GET, handleClear);
  server.begin();

  Serial.println("HTTP server started");
  Serial.println("Download CSV at:");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/data.csv");
}

// -------- LOOP --------
void loop() {
  server.handleClient();

  if (millis() - lastLogTime >= logInterval) {
    lastLogTime = millis();
    time_t now = time(nullptr);
    float value = readSensor();
    appendCSV(now, value);
    Serial.printf("Logged: %ld , %.2f\n", now, value);
  }
}
