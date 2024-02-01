#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SocketIOclient.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <AccelStepper.h>
// #include <MultiStepper.h>
#include <PCF8574.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>


boolean setupWifi(const char* ssid, const char* password);
void socketConnect();
void readTemperature();
void readHumidity();
void readRain();
void controlRoof(bool status);
void readSsid(char* ssid);
void readPassword(char* password);
void createAP();
void createWebserver();
void writeSsid(const char* ssid);
void writePassword(const char* password);
void checkWifi();
void sioEvent(socketIOmessageType_t type, uint8_t * payload, size_t length);
void controlRoof(bool status);
void setupRoof();
void readData();

const char* payload = "[\"sensorData\",\"Hello from ESP8266!\"]";
WiFiClient client;
const char* HOST = "famnkunf.xyz";
const int PORT = 5555;
char ssid[32];
char password[32];
ESP8266WebServer server(80);

SocketIOclient sio;
unsigned long lastMillis = 0;

String eventPayload;
String eventName;
String eventData;
int startIndex;
int endIndex;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

int temperature;
int humidity;
int pressure;
bool rain=false;
bool roofStatus=false;
String data;

// Side Motor
const int dirPin2 = D7;
const int stepPin2 =D6;
const int dirPin1 = D4;
const int stepPin1 = D5;
const int servoPin = D3;
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
Servo servo;
const int speed = 200;

// Roof motor
// const int dirPin2 = D2;
// const int stepPin2 =D1;
// AccelStepper stepper2(AccelStepper::FULL4WIRE, stepPin2, dirPin2);

// MultiStepper steppers;

int expanderAddress = 0x20;
PCF8574 expander(expanderAddress);
PCF8574::DigitalInput exPinStatus;

int rainSensorPin = 0;
Adafruit_BME280 bme;

const char* indexPage = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Credential Entry</title>
</head>
<body>
    <h1>WiFi Credential Entry</h1>
    <form action="/save" method="POST">
        <label for="ssid">SSID:</label>
        <input type="text" id="ssid" name="ssid" required><br><br>
        <label for="password">Password:</label>
        <input type="password" id="password" name="password" required><br><br>
        <input type="submit" value="Save">
    </form>
</body>
</html>
)=====";

void setup() {
  EEPROM.begin(64);
  Serial.begin(115200);
  Wire.begin();
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(servoPin, OUTPUT);
  expander.pinMode(P2, INPUT);
  stepper1.setMaxSpeed(5000);
  stepper1.setAcceleration(1000);
  stepper1.setMinPulseWidth(100);
  stepper2.setMaxSpeed(5000);
  stepper2.setAcceleration(1000);
  stepper2.setMinPulseWidth(100);
  // servo.attach(servoPin);
  setupRoof();
  while(!bme.begin(0x76)){
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure();
  Serial.println("BME280 detected!");
  checkWifi();
  timeClient.begin();
  timeClient.setTimeOffset(25200); // Set time offset to GMT+7 (7 * 3600 seconds)
}

void loop() {
  server.handleClient();
  sio.loop();

  if (millis() - lastMillis > 2000 && sio.isConnected() == true) {
    // sio.sendEVENT(payload);
    lastMillis = millis();
    rain = expander.digitalRead(P2);
    exPinStatus = expander.digitalReadAll();
    Serial.print(exPinStatus.p0);
    Serial.print(exPinStatus.p1);
    Serial.print(exPinStatus.p2);
    Serial.print(exPinStatus.p3);
    Serial.print(exPinStatus.p4);
    Serial.print(exPinStatus.p5);
    Serial.print(exPinStatus.p6);
    Serial.println(exPinStatus.p7);
    if (rain == true && roofStatus == true){
      roofStatus = false;
      controlRoof(false);
    }
  }
  timeClient.update();
}

void handleRoot() {
  server.send(200, "text/html", indexPage);
}

void handleSave() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");

  Serial.println("Saving credentials");
  Serial.println(ssid);
  Serial.println(password);
  writeSsid(ssid.c_str());
  writePassword(password.c_str());
  checkWifi();
  server.send(200, "text/plain", "Credentials saved successfully");
}

void createWebserver() {
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.begin();
  Serial.println("HTTP server started");
}
  


boolean setupWifi(const char* ssid, const char* password) {
  Serial.print(ssid);
  Serial.print(" ");
  Serial.println(password);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  Serial.println(WiFi.macAddress());
  int reconnectCounter = 0;
  while (WiFi.status() != WL_CONNECTED && reconnectCounter < 20) {
    Serial.print('.');
    delay(2000);
    reconnectCounter++;
  }
  if (reconnectCounter >= 10) {
    Serial.println("Connection failed");
    return false;
  }
  Serial.print("\n");
  Serial.println(WiFi.localIP());
  return true;
}

void socketConnect(){
  sio.beginSSL(HOST, PORT, "/socket.io/?EIO=4");
  sio.onEvent(sioEvent);
  sio.setReconnectInterval(5000);
}

void readSsid(char* ssid) {
  for (int i = 0; i < 32; ++i) {
    ssid[i] = EEPROM.read(i);
  }
}

void readPassword(char* password) {
  for (int i = 0; i < 32; ++i) {
    password[i] = EEPROM.read(i + 32);
  }
}

void createAP() {
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP("BMTObservatory");
  Serial.println("AP created");
  Serial.println(WiFi.softAPIP());
}

void writeSsid(const char* ssid) {
  for (int i = 0; i < 32; ++i) {
    EEPROM.write(i, ssid[i]);
  }
  EEPROM.commit();
}

void writePassword(const char* password) {
  for (int i = 0; i < 32; ++i) {
    EEPROM.write(i + 32, password[i]);
  }
  EEPROM.commit();
}

void checkWifi(){
  readSsid(ssid);
  readPassword(password);
  if (!setupWifi(ssid, password)) {
    Serial.println("Failed to connect to WiFi");
    Serial.println(ssid);
    Serial.println(password);
    createAP();
    delay(5000);
    createWebserver();
  } else {
    socketConnect();
  }
}

void sioEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      sio.send(sIOtype_CONNECT, "/");
      sio.sendEVENT("[\"clientType\", \"device\"]");
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      break;
    case sIOtype_EVENT:
      Serial.printf("[WSc] get event: %s\n", payload);
      // Extracting the event name from the payload
      eventPayload= String((char*)payload);
      startIndex = eventPayload.indexOf('[') + 2;
      endIndex = eventPayload.indexOf(',')-1;
      eventName = String(eventPayload.substring(startIndex, endIndex));

      // Extracting the event data from the payload
      eventData = String(eventPayload.substring(endIndex + 3, eventPayload.length() - 2));
      Serial.printf("Event name: %s\n", eventName.c_str());
      Serial.printf("Event data: %s\n", eventData.c_str());
      if (eventName == "requestData"){
        readData();
        data = "{\"temperature\":" + String(temperature) + ",\"pressure\":" + String(pressure) + ",\"humidity\":" + String(humidity) + ",\"rain\":" + String(rain) + ",\"roof\":" + String(roofStatus) + "}";
        data = "[\"sensorData\"," + data + "]";
        sio.sendEVENT(data);
        Serial.println(data);
      } else if (eventName == "roofStatus"){
        roofStatus = eventData.toInt();
        controlRoof(roofStatus);
        Serial.printf("Change roof status to: %s", eventData.c_str());
      }
      
      break;
    case sIOtype_ACK:
      Serial.printf("[WSc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[WSc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      Serial.printf("[WSc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      Serial.printf("[WSc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
    default:
      Serial.printf("Invalid WStype [%d]\n", type);
      break;
  }
}


void controlRoof(bool status) {
  if (status == true) {
    Serial.println("Opening roof");
    stepper1.setSpeed(speed);
    lastMillis = millis();
    while (true){
      if (millis() - lastMillis > 200) {
        lastMillis = millis();
        if(expander.digitalRead(P1) == 1){
          Serial.println("SideRoof opened");
          break;
        }
      }
      stepper1.runSpeed();
      yield();
   }
   stepper2.setSpeed(speed);
   lastMillis = millis();
   while (true){
      if (millis() - lastMillis > 200) {
        lastMillis = millis();
        if(expander.digitalRead(P3) == 1){
          Serial.println("Roof opened");
          break;
        }
      }
      stepper2.runSpeed();
      yield();
   }
   servo.write(90);
   sio.sendEVENT("[\"node.roofReady\", true]");
  } else {
    Serial.println("Closing roof");
    stepper1.setSpeed(-speed);
    lastMillis = millis();
    while (true){
      if (millis() - lastMillis > 200) {
        // sio.sendEVENT(payload);
        lastMillis = millis();
        if(expander.digitalRead(P0) == 1){
          Serial.println("SideRoof closed");
          break;
        }
      }
      stepper1.runSpeed();
      yield();
    }
    while (true)
    {
      if (millis() - lastMillis > 200) {
        lastMillis = millis();
        if(expander.digitalRead(P4) == 1){
          Serial.println("Roof closed");
          break;
        }
      }
      stepper2.runSpeed();
      yield();
    }
    servo.write(0);
    sio.sendEVENT("[\"node.roofReady\", false]");
  }
}

void readRain(){
  rain = !expander.digitalRead(P2);
  Serial.print("Rain: ");
  Serial.print(rain);
}

void readTemperature(){
  temperature = bme.readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);
}

void readHumidity(){
  humidity = bme.readHumidity();
  Serial.print("Humidity: ");
  Serial.println(humidity);
}

void readPressure(){
  pressure = bme.readPressure()/100;
  Serial.print("Pressure: ");
  Serial.println(pressure);
}

void setupRoof(){
  Serial.println("Setting up roof");
  expander.begin();
  expander.pinMode(P0, INPUT);
  expander.pinMode(P1, INPUT);
  expander.pinMode(P3, INPUT);
  expander.pinMode(P4, INPUT);
  exPinStatus = expander.digitalReadAll();
  if (exPinStatus.p0 == 0 && exPinStatus.p1 == 0 ){
    stepper1.setSpeed(-speed);
    while (true){
      // exPinStatus = expander.digitalReadAll();
      if (exPinStatus.p0 == 1){
        Serial.println("SideRoof closed");
        break;
      }
      stepper1.runSpeed();
      yield();
    }
  }
  if (exPinStatus.p3 == 0 && exPinStatus.p4 == 0 ){
    stepper2.setSpeed(speed);
    while (true){
      exPinStatus = expander.digitalReadAll();
      if (exPinStatus.p3 == 1){
        Serial.println("Roof closed");
        break;
      }
      stepper2.runSpeed();
      yield();
    }
    roofStatus = false;
  }
  servo.write(0);
}


void readData(){
  readHumidity();
  readTemperature();
  readPressure();
  readRain();
}