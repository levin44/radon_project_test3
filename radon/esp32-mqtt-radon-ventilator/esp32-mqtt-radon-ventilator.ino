//Created by Nisal - v1.0.5
// Radon ventilator

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// Sine Wave Simulation Configuration
const float SINE_WAVE_MIN_VALUE = 10.0;    // Lowest value of the sine wave
const float SINE_WAVE_MAX_VALUE = 50.0;    // Highest value of the sine wave
const float SINE_WAVE_INTERVAL = 60.0;     // Period in seconds for one complete cycle

// Wi-Fi credentials
const char* ssid = "Nebula";
const char* password = "2024niDI";

// MQTT broker settings
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* publishTopicSystemStatus = "radoncontrol/status";
const char* publishTopicRelayStatus = "radoncontrol/relaystatus";
const char* subscribeTopicRelay = "radoncontrol/relay";
const char* subscribeTopicMode = "radoncontrol/mode";
const char* subscribeTopicRadonValue = "radonvalue/sensor1";
const char* subscribeTopicRadonValue2 = "radonvalue/sensor2";
const char* subscribeTopicRadonValue3 = "radonvalue/sensor3";


// Pin definitions
const int relayPin = 2;
const int relayLEDPin = 22;

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

// Latest radon values per sensor
float lastRadonSensor1 = 0.0;
float lastRadonSensor2 = 0.0;
float lastRadonSensor3 = 0.0;

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
void handleRadonValue(String topic, String message);
void updateRelayState();



void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set pin modes
  pinMode(relayPin, OUTPUT);
  pinMode(relayLEDPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Initialize relay OFF
  digitalWrite(relayLEDPin, LOW);  // Initialize relay indicator led OFF

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
      mqttClient.subscribe(subscribeTopicRadonValue2);
      mqttClient.subscribe(subscribeTopicRadonValue3);
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
  } else if (String(topic) == subscribeTopicRadonValue ||
             String(topic) == subscribeTopicRadonValue2 ||
             String(topic) == subscribeTopicRadonValue3) {
    handleRadonValue(String(topic), messageTemp);
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

void handleRadonValue(String topic, String message) {
  // Parse the radon value from the message
  float radonValue = message.toFloat();

  // Track last reading per sensor
  if (topic == subscribeTopicRadonValue) {
    lastRadonSensor1 = radonValue;
  } else if (topic == subscribeTopicRadonValue2) {
    lastRadonSensor2 = radonValue;
  } else if (topic == subscribeTopicRadonValue3) {
    lastRadonSensor3 = radonValue;
  }

  Serial.print("Received ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(radonValue);
    
  // In auto mode, control relay based on any sensor exceeding threshold
  if (currentMode == "auto") {
    bool anyHigh = (lastRadonSensor1 > highRadonThreshold) ||
                   (lastRadonSensor2 > highRadonThreshold) ||
                   (lastRadonSensor3 > highRadonThreshold);

    relayState = anyHigh;
    if (anyHigh) {
      Serial.println("Auto mode: High radon detected on at least one sensor, turning relay ON");
    } else {
      Serial.println("Auto mode: All sensors normal, turning relay OFF");
    }
    updateRelayState();
  }
}

void updateRelayState() {
// Only update relay in manual mode or when explicitly commanded

digitalWrite(relayPin, relayState ? HIGH : LOW);
digitalWrite(relayLEDPin, relayState ? HIGH : LOW);
Serial.print("Relay is ");
Serial.println(relayState ? "ON" : "OFF");
Serial.print("Publishing RelayStatus: ");
Serial.println(relayState ? "ON" : "OFF");
relayState ? mqttClient.publish(publishTopicRelayStatus, "on") : mqttClient.publish(publishTopicRelayStatus, "off");
}
