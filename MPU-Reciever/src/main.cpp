#include <WiFi.h>

const char* ssid = "ESP32_Direct";
const char* password = "password123";
const char* host = "192.168.4.1";  // Default SoftAP IP
const uint16_t port = 8080;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  
  // Connect to the SoftAP
  Serial.println("Connecting to " + String(ssid));
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("Connected to SoftAP!");
  
  // Connect to the server
  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    return;
  }
  
  Serial.println("Connected to server!");
}

void loop() {
  // Check if the server sent data
  if (client.available()) {
    String line = client.readStringUntil('\n');
    Serial.println("Received: " + line);
  }
  
  // Send a command occasionally
  if (millis() % 5000 < 10) {
    client.println("STATUS");
  }
  
  delay(10);
}