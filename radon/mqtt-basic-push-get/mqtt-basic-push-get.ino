//Created by Nisal - v1.0.1
//Basic MQTT publish subcribe code

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi credentials
const char* ssid = "NOVA";
const char* password = "NOVA22NM";

// MQTT broker settings
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* publishTopicRadonValue = "radonvalue/sensor1";
const char* subscribeTopicLED = "radoncontrol/led";


// Pin definitions
const int ledPin = 2;

// Timing variables
long lastMsg = 0;
const long msgInterval = 5000; // 5 seconds

// Publish Data variable
float publishDataValue = 0.0;

// Wi-Fi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Function declarations
void setupWiFi();
void setupMQTT();
void reconnect();
void callback(char* topic, byte* message, unsigned int length);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set pin modes
  pinMode(ledPin, OUTPUT);

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
      mqttClient.subscribe(subscribeTopicLED);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishData() {
  // Simulate publishDataValue reading
  publishDataValue = 35.89; // 

  // Convert publishDataValue to char array
  char dataString[8];
  dtostrf(publishDataValue, 1, 2, dataString);

  Serial.print("Publishing Data Value: ");
  Serial.println(dataString);

  mqttClient.publish(publishTopicRadonValue, dataString);
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

  // Control the LED based on the message
  if (messageTemp == "on") {
    digitalWrite(ledPin, HIGH);
    Serial.println("LED is ON");
  } else if (messageTemp == "off") {
    digitalWrite(ledPin, LOW);
    Serial.println("LED is OFF");
  }
}
