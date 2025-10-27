//Created by Nisal - v1.0.3
// Radon ventilator
//Basic MQTT publish subcribe code

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// Sine Wave Simulation Configuration
const float SINE_WAVE_MIN_VALUE = 10.0;    // Lowest value of the sine wave
const float SINE_WAVE_MAX_VALUE = 50.0;    // Highest value of the sine wave
const float SINE_WAVE_INTERVAL = 60.0;     // Period in seconds for one complete cycle

// Wi-Fi credentials
const char* ssid = "NOVA";
const char* password = "NOVA22NM";

// MQTT broker settings
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* publishTopicSystemStatus = "radoncontrol/status";
const char* publishTopicRelayStatus = "radoncontrol/relaystatus";
const char* subscribeTopicRelay = "radoncontrol/relay";
const char* subscribeTopicMode = "radoncontrol/mode";
const char* subscribeTopicRadonValue = "radonvalue/sensor1";


// Pin definitions
const int relayPin = 2;

// Threshold for high radon
float highRadonThreshold = 30.0;

// Mode control variables
String currentMode = "auto";  // Default to auto mode
bool relayState = false;        // Track relay state

// Timing variables
long lastMsg = 0;
const long msgInterval = 5000; // 5 seconds

// Publish Data variable
String publishDataValue = "";

// Wi-Fi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Function declarations
void setupWiFi();
void setupMQTT();
void reconnect();
void callback(char* topic, byte* message, unsigned int length);
void handleRelayControl(String message);
void handleModeControl(String message);
void handleRadonValue(String message);
void updateRelayState();



void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set pin modes
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Initialize relay OFF

  // Connect to Wi-Fi
  setupWiFi();

  // Setup MQTT
  setupMQTT();
}

void loop() {
  // Reconnect to MQTT if disconnected
  if (!mqttClient.connected()) {
    reconnect();
  }

  // Handle incoming MQTT messages
  mqttClient.loop();

  // Publish  data at regular intervals
  long now = millis();
  if (now - lastMsg > msgInterval) {
    lastMsg = now;
    publishData();
  }
}

void setupWiFi() {
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
}

void reconnect() {
  // Reconnect to MQTT broker
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT Broker...");

    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT Broker.");
      mqttClient.subscribe(subscribeTopicRelay);
      mqttClient.subscribe(subscribeTopicMode);
      mqttClient.subscribe(subscribeTopicRadonValue);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishData() {
  // publish system online
  publishDataValue = "Control System Online";

  Serial.print("Publishing Data : ");
  Serial.println(publishDataValue);

  mqttClient.publish(publishTopicSystemStatus, publishDataValue.c_str());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  String messageTemp;
  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // Handle different topics
  if (String(topic) == subscribeTopicRelay) {
    handleRelayControl(messageTemp);
  } else if (String(topic) == subscribeTopicMode) {
    handleModeControl(messageTemp);
  } else if (String(topic) == subscribeTopicRadonValue) {
    handleRadonValue(messageTemp);
  }
}

void handleRelayControl(String message) {

if (currentMode == "manual"){
    if (message == "on") {
        relayState = true;
        Serial.println("Relay command: ON");
        } else if (message == "off") {
        relayState = false;
        Serial.println("Relay command: OFF");
        }
}
  updateRelayState();
}

void handleModeControl(String message) {
  if (message == "auto" || message == "manual") {
    currentMode = message;
    Serial.print("Mode changed to: ");
    Serial.println(currentMode);
  }
}

void handleRadonValue(String message) {
  // Parse the radon value from the message
  float radonValue = message.toFloat();
  Serial.print("Received radon value: ");
  Serial.println(radonValue);
    
  // In auto mode, control relay based on radon levels
  if (currentMode == "auto") {
    if (radonValue > highRadonThreshold) {  // Threshold for high radon
      relayState = true;
      Serial.println("Auto mode: High radon detected, turning relay ON");
    } else {
      relayState = false;
      Serial.println("Auto mode: Normal radon levels, turning relay OFF");
    }
    updateRelayState();
  }
}

void updateRelayState() {
// Only update relay in manual mode or when explicitly commanded

digitalWrite(relayPin, relayState ? HIGH : LOW);
Serial.print("Relay is ");
Serial.println(relayState ? "ON" : "OFF");
Serial.print("Publishing RelayStatus: ");
Serial.println(relayState ? "ON" : "OFF");
relayState ? mqttClient.publish(publishTopicRelayStatus, "on") : mqttClient.publish(publishTopicRelayStatus, "off");
}
