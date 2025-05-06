#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// Pin setup
#define TDS_PIN 35
#define PH_SENSOR_PIN 34
#define ONE_WIRE_BUS 4
#define TURBIDITY_SENSOR_PIN 32

// WiFi config
const char* ssid = "Galaxy S23 Ultra 2609";
const char* password = "alohomora";

// ThingSpeak settings
const char* mqtt_server = "mqtt3.thingspeak.com";
const int mqtt_port = 1883;
const char* mqtt_clientID = "LS0aBxE3OTsJGggOITk0Bhw";
const char* mqtt_username = "LS0aBxE3OTsJGggOITk0Bhw";
const char* mqtt_password = "NY8+pfL/8G1dHH+J8x3CstOy";
const char* mqtt_topic = "channels/2926233/publish";

// Components init
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);

byte verticalLine[8] = {B00100, B00100, B00100, B00100, B00100, B00100, B00100, B00100};

// Water quality parameters
float tds = 0.0;
float temp = 0.0;
float pH = 0.0;
float turbidity = 0.0;
float wqi = 0.0;

// System state
bool waterOK = true;
bool sensorError = false;
int unsafeCount = 0;

// Timing variables
int displayMode = 0;
unsigned long lastUpdateTime = 0;
const unsigned long displayInterval = 2000;
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 15000;
unsigned long lastUnsafeTime = 0;

// For sensor error detection
float lastTDS[10], lastPH[10], lastTurb[10];
int tdsIdx = 0, phIdx = 0, turbIdx = 0;

// Takes multiple readings to reduce noise
float averageAnalogRead(int pin, int samples = 10) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(10);
  }
  return sum / samples;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n===== Water Quality Monitor Starting =====");
  
  analogReadResolution(12);
  
  sensors.begin();
  
  Wire.begin();
  
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.createChar(0, verticalLine);
  
  // Welcome message
  lcd.setCursor(0, 0); 
  lcd.print("Water Quality");
  lcd.setCursor(0, 1); 
  lcd.print("Monitor v1.0");
  delay(2000);
  
  setupWiFi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("WiFi Connected");
  lcd.setCursor(0, 1); 
  lcd.print("IP: ");
  lcd.print(WiFi.localIP());
  delay(2000);
  
  lcd.clear();
  
  Serial.println("Setup complete - starting monitoring");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Wait after unsafe water detection
  if (millis() - lastUnsafeTime < 120000) {
    return;
  }
  
  // Read all sensors
  tds = readTDSSensor();
  pH = readPHSensor();
  temp = readTemperatureSensor();
  turbidity = readTurbiditySensor();
  
  wqi = calculateWQI(tds, pH, turbidity);
  
  sensorError = checkStuckSensor(tds, pH, turbidity);
  
  // Check if water is safe
  bool currentSafe = (tds < 500 && pH >= 6.5 && pH <= 7.5 && turbidity < 5.0 && !sensorError);
  
  if (!currentSafe) {
    unsafeCount++;
    Serial.println("Unsafe reading detected. Count: " + String(unsafeCount));
  } else {
    unsafeCount = 0;
  }
  
  // Need 5 unsafe readings to confirm
  waterOK = (unsafeCount < 5);
  
  if (!waterOK || sensorError) {
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("Water Unsafe!");
    lcd.setCursor(0, 1); 
    
    if (sensorError) {
      lcd.print("Sensor Error!");
      Serial.println("Sensor error detected. Waiting for 2 minutes.");
    } else {
      lcd.print("Unsafe Confirmed");
      Serial.println("Confirmed unsafe water. Waiting for 2 minutes.");
    }
    
    lastUnsafeTime = millis();
    delay(1000);
    return;
  }
  
  // Rotate display info
  if (millis() - lastUpdateTime >= displayInterval) {
    updateDisplay();
    displayMode = (displayMode + 1) % 5;
    lastUpdateTime = millis();
  }
  
  // Send to cloud
  if (millis() - lastPublishTime >= publishInterval) {
    publishData();
    lastPublishTime = millis();
  }
}

void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Not used but required
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(mqtt_clientID, mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, error code=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

float readTDSSensor() {
  float analogValue = averageAnalogRead(TDS_PIN);
  
  float voltage = analogValue * 3.3 / 4095.0;
  
  // Calibration formula
  float tdsValue = 12.91 + 0.670 * analogValue - 0.001187 * pow(analogValue, 2) +
                  9.02e-7 * pow(analogValue, 3) - 1.665e-10 * pow(analogValue, 4);
  
  lastTDS[tdsIdx++ % 10] = tdsValue;
  
  Serial.print("TDS: ");
  Serial.print(tdsValue, 1);
  Serial.println(" ppm");
  
  return tdsValue;
}

float readPHSensor() {
  float analogValue = averageAnalogRead(PH_SENSOR_PIN);
  
  // pH calibration curve
  float pHValue = 3.787 * log(89.301 * analogValue + 1) - 38.317;
  
  lastPH[phIdx++ % 10] = pHValue;
  
  Serial.print("pH: ");
  Serial.println(pHValue, 1);
  
  return pHValue;
}

float readTemperatureSensor() {
  sensors.requestTemperatures();
  
  float tempValue = sensors.getTempCByIndex(0);
  
  if (tempValue == -127.0) {
    Serial.println("Temperature sensor error");
    return 0;
  }
  
  Serial.print("Temperature: ");
  Serial.print(tempValue, 1);
  Serial.println("°C");
  
  return tempValue;
}

float readTurbiditySensor() {
  float sensorValue = averageAnalogRead(TURBIDITY_SENSOR_PIN);
  
  lastTurb[turbIdx++ % 10] = sensorValue;
  
  Serial.print("Turbidity: ");
  Serial.print(sensorValue, 1);
  Serial.println("%");
  
  return sensorValue;
}

float calculateWQI(float tds, float pH, float turbidity) {
  // Score each parameter
  float tds_score = (tds <= 300) ? 100 : (tds <= 500) ? 70 : 40;
  float pH_score = (pH >= 6.5 && pH <= 8.5) ? 100 : (pH >= 6.0 && pH <= 9.0) ? 70 : 40;
  float turb_score = (turbidity < 5.0) ? 100 : (turbidity < 10.0) ? 70 : 40;
  
  float waterQualityIndex = (tds_score + pH_score + turb_score) / 3.0;
  
  Serial.print("Water Quality Index: ");
  Serial.print(waterQualityIndex, 1);
  Serial.println("/100");
  
  return waterQualityIndex;
}

bool checkStuckSensor(float tds, float pH, float turbidity) {
  // Need 10 readings to check
  if (tdsIdx < 10 || phIdx < 10 || turbIdx < 10) {
    return false;
  }
  
  // Check for non-changing TDS readings
  bool tdsStuck = true;
  float firstTDS = lastTDS[0];
  for (int i = 1; i < 10; i++) {
    if (abs(lastTDS[i] - firstTDS) > 1.0) {
      tdsStuck = false;
      break;
    }
  }
  
  // Check for non-changing pH readings
  bool phStuck = true;
  float firstPH = lastPH[0];
  for (int i = 1; i < 10; i++) {
    if (abs(lastPH[i] - firstPH) > 0.1) {
      phStuck = false;
      break;
    }
  }
  
  // Check for non-changing turbidity readings
  bool turbStuck = true;
  float firstTurb = lastTurb[0];
  for (int i = 1; i < 10; i++) {
    if (abs(lastTurb[i] - firstTurb) > 0.5) {
      turbStuck = false;
      break;
    }
  }
  
  // Check for impossible readings
  bool extremeReadings = (tds > 1500 || tds < 5 || pH > 14 || pH < 0 || turbidity > 100);
  
  return (tdsStuck || phStuck || turbStuck || extremeReadings);
}

void updateDisplay() {
  lcd.clear();
  
  // Status always on top
  lcd.setCursor(0, 0);
  lcd.print("Status: ");
  lcd.print(waterOK ? "GOOD" : "BAD");
  
  lcd.setCursor(0, 1);
  
  switch (displayMode) {
    case 0:
      lcd.print("TDS: ");
      lcd.print(tds, 1);
      lcd.print(" ppm");
      break;
    case 1:
      lcd.print("pH: ");
      lcd.print(pH, 1);
      break;
    case 2:
      lcd.print("Temp: ");
      lcd.print(temp, 1);
      lcd.print(" C");
      break;
    case 3:
      lcd.print("Turb: ");
      lcd.print(turbidity, 1);
      lcd.print("%");
      break;
    case 4:
      lcd.print("WQI: ");
      lcd.print(wqi, 1);
      lcd.print("/100");
      break;
  }
}

void publishData() {
  if (!client.connected()) {
    reconnect();
  }
  
  // Format: field1=value1&field2=value2&...
  String dataMessage = "field1=" + String(tds) + 
                      "&field2=" + String(pH) + 
                      "&field3=" + String(temp) + 
                      "&field4=" + String(turbidity) + 
                      "&field5=" + String(wqi) + 
                      "&field6=" + String(waterOK ? 1 : 0) + 
                      "&status=MQTTPUBLISH";
  
  char payload[200];
  dataMessage.toCharArray(payload, 200);
  
  Serial.print("Publishing data... ");
  if (client.publish(mqtt_topic, payload)) {
    Serial.println("Success!");
  } else {
    Serial.println("Failed!");
  }
}
