//Created by Nisal - v1.0.6
//ESP32 MQTT Radon Sensor3 

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <Rd200m.h>
#include <LiquidCrystal.h>

//Radon initialize
#define INTERVAL 60  //sec
Rd200m radon(&Serial2);

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(19, 23, 18, 1, 3, 15);

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
const char* publishTopicRadonValue = "radonvalue/sensor3";
const char* subscribeTopicMode = "radonvalue/mode3";

// Mode
String currentMode = "real"; //default mode is real


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
float generateSimulatedValues();

//Radon get value
void getit() {
  Serial.print(String(radon.value()) + String(" pCi"));
  String s = radon.status();
  if (s != "") Serial.println(String(" (")+ s +String(")"));
  else Serial.println();
}

// Sine Wave Simulation Function
float generateSimulatedValues() {
  // Get current time in milliseconds
  unsigned long currentTime = millis();
  
  // Convert to seconds
  float timeInSeconds = currentTime / 1000.0;
  
  // Calculate the phase of the sine wave (0 to 2π for one complete cycle)
  float phase = (timeInSeconds / SINE_WAVE_INTERVAL) * 2.0 * PI;
  
  // Generate sine wave value between -1 and 1
  float sineValue = sin(phase);
  
  // Scale and offset to fit between min and max values
  float amplitude = (SINE_WAVE_MAX_VALUE - SINE_WAVE_MIN_VALUE) / 2.0;
  float offset = SINE_WAVE_MIN_VALUE + amplitude;
  
  return sineValue * amplitude + offset;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize LCD display
  lcd.begin(16, 2);
  lcd.write("Initializing...");
  delay(1000);

  //Radon setup
  Serial2.begin(19200, SERIAL_8N1, 16, 17);
  Serial.println("\nRadon Sensor RD200M V1.0 April 10, 2018");
  radon.debug(0);
  radon.onPacket(getit);

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

  //Radon get value
  updateRadonValue();

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
      mqttClient.subscribe(subscribeTopicMode);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishData() {
  if(currentMode == "real"){
    publishDataValue = radon.value() ? radon.value()*37: 0.0;
  }else if(currentMode == "simulated"){
    // Generate sine wave simulated data
    publishDataValue = generateSimulatedValues();
  }

  //printing on the LCD display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Radon Value :");
  lcd.setCursor(0, 1);
  lcd.print(publishDataValue);
  lcd.setCursor(7, 1);
  lcd.write("Bq");
  // delay(1000);

  // Convert publishDataValue to char array
  char dataString[8];
  dtostrf(publishDataValue, 1, 2, dataString);

  Serial.print("Publishing Data Value: ");
  Serial.println(dataString);

  //publishing data via mqtt
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

  // Control the Mode based on the message
  if (messageTemp == "real") {
    currentMode = "real";
    Serial.println("Mode changed to : real");
  } else if (messageTemp = "simulated") {
    currentMode = "simulated";
    Serial.println("Mode changed to : simulated");
  }
}

void updateRadonValue(){
  static unsigned mark = 0;
  
  if (millis() > mark) {
	  mark = millis() + (INTERVAL * 1000);
	  radon.request();
  }
  radon.update();
}
