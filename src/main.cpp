#include <Arduino.h>
#include <ESP8266WiFi.h>


void setupWifi(const char* ssid, const char* password);

const char* message = "Hello";

void setup() {
  Serial.begin(115200);
  setupWifi("famnkunf", "famnkunf123");
}

void loop() {
  delay(1000);
  Serial.println("====================================");
}


void setupWifi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print("\n");
  Serial.println(WiFi.localIP());
}

