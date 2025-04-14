/**
 * @file BMI088_Example.ino
 * @brief Example sketch for BMI088 IMU driver
 * @author Claude
 * @date April 2025
 * 
 * This example initializes the BMI088 IMU and continuously reads
 * accelerometer and gyroscope data, printing the values to Serial.
 */

#include <Wire.h>
#include "BMI088.h"

// Define I2C pins for ESP32C6 Mini (adjust these for your specific board)
#define SDA_PIN 22  // Change to match your board
#define SCL_PIN 23  // Change to match your board

// Create BMI088 instance with custom I2C pins
BMI088 imu(Wire, SDA_PIN, SCL_PIN);

// Variables to store sensor readings
float ax, ay, az;  // Accelerometer data in g
float gx, gy, gz;  // Gyroscope data in degrees/second
float temperature; // Temperature in Celsius

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for serial port to connect
  
  Serial.println("BMI088 IMU Example");
  
  // Initialize BMI088 with custom configuration
  if (!imu.begin(
        BMI088_ACC_RANGE_12G,     // ±12g accelerometer range
        BMI088_ACC_ODR_400,       // 400Hz output data rate
        BMI088_ACC_BW_NORMAL,     // Normal bandwidth
        BMI088_GYRO_RANGE_2000,   // ±2000°/s gyroscope range
        BMI088_GYRO_BW_ODR_400_47 // 400Hz ODR, 47Hz bandwidth
      )) {
    Serial.println("Failed to initialize BMI088!");
    while (1) delay(10);  // Halt if initialization failed
  }
  
  Serial.println("BMI088 initialized successfully!");
  
  // Check chip IDs
  if (imu.checkAccConnection()) {
    Serial.println("Accelerometer OK");
  } else {
    Serial.println("Accelerometer not detected!");
  }
  
  if (imu.checkGyroConnection()) {
    Serial.println("Gyroscope OK");
  } else {
    Serial.println("Gyroscope not detected!");
  }
  
  delay(100);
}

void loop() {
  // Read accelerometer data in g units
  if (imu.readAccelerometer(&ax, &ay, &az)) {
    Serial.print("Acceleration (g): ");
    Serial.print(ax, 2); Serial.print(", ");
    Serial.print(ay, 2); Serial.print(", ");
    Serial.print(az, 2); Serial.println();
  }
  
  // Read gyroscope data in degrees/second
  if (imu.readGyroscope(&gx, &gy, &gz)) {
    Serial.print("Gyroscope (deg/s): ");
    Serial.print(gx, 2); Serial.print(", ");
    Serial.print(gy, 2); Serial.print(", ");
    Serial.print(gz, 2); Serial.println();
  }
  
  // Read temperature
  temperature = imu.readTemperature();
  Serial.print("Temperature (°C): ");
  Serial.println(temperature, 2);
  
  Serial.println();
  delay(500);  // Update every 500ms
}