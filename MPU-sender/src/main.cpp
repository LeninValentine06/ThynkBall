#include <WiFi.h>

// SoftAP Configuration
const char* ssid = "ESP32_Direct";
const char* password = "password123";
const char* host = "192.168.4.1";  // Default SoftAP IP
const uint16_t port = 8080;

WiFiClient client;
unsigned long lastConnectionAttempt = 0;
const int connectionInterval = 5000;  // Try to reconnect every 5 seconds
unsigned long lastCommandSent = 0;
const int commandInterval = 5000;     // Send command every 5 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give serial monitor time to connect
  
  Serial.println("\n\n=== ESP32 MPU6050 Client ===");
  
  // Disconnect from any previous networks
  WiFi.disconnect(true);
  delay(1000);
  
  // Set to station mode (client)
  WiFi.mode(WIFI_STA);
  delay(1000);
  
  connectToServer();
}

void loop() {
  // If WiFi is disconnected, try to reconnect
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastConnectionAttempt > connectionInterval) {
      Serial.println("WiFi disconnected, reconnecting...");
      connectToServer();
      lastConnectionAttempt = millis();
    }
  } 
  // If WiFi is connected but client is disconnected, try to reconnect
  else if (!client.connected()) {
    if (millis() - lastConnectionAttempt > connectionInterval) {
      Serial.println("Reconnecting to server...");
      if (client.connect(host, port)) {
        Serial.println("Reconnected to server!");
      } else {
        Serial.println("Failed to reconnect to server");
      }
      lastConnectionAttempt = millis();
    }
  } 
  // If connected, process data and send commands
  else {
    // Read data if available
    while (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println("Received: " + line);
    }
    
    // Send command periodically
    if (millis() - lastCommandSent > commandInterval) {
      client.println("STATUS");
      Serial.println("Sent: STATUS");
      lastCommandSent = millis();
    }
  }
  
  delay(10);  // Small delay to prevent CPU hogging
}

void connectToServer() {
  // Connect to SoftAP
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  int timeout = 20;  // 10 seconds timeout
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(500);
    Serial.print(".");
    timeout--;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Connect to server
    Serial.print("Connecting to host ");
    Serial.println(host);
    
    if (client.connect(host, port)) {
      Serial.println("Connected to server!");
    } else {
      Serial.println("Connection to server failed");
    }
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed");
  }
  
  lastConnectionAttempt = millis();
}