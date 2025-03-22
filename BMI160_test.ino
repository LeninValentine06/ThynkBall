#include <BMI160Gen.h>            //https://github.com/hanyazou/BMI160-Arduino
#include <Wire.h>
 
// I2C Configuration for ESP32
const int i2c_addr = 0x68;  // Default I2C address for BMI160
const int sda_pin = 21;     // I2C SDA Pin for ESP32 (default GPIO 21)
const int scl_pin = 22;     // I2C SCL Pin for ESP32 (default GPIO 22)
 
void setup() {
  // Initialize Serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to connect (not required on ESP32)
 
  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(sda_pin, scl_pin);
 
  // Initialize the BMI160 device in I2C mode
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr)) {
    Serial.println("BMI160 initialization failed!");
    while (1); // Halt if initialization fails
  }
 
  Serial.println("BMI160 initialized successfully in I2C mode!");
}
 
void loop() {
  int gx, gy, gz; // Raw gyroscope values
  int ax, ay, az; // Raw accelerometer values
 
  // Read raw gyroscope measurements from the BMI160
  BMI160.readGyro(gx, gy, gz);
 
  // Read raw accelerometer measurements from the BMI160
  BMI160.readAccelerometer(ax, ay, az);
 
  // Display the gyroscope values (X, Y, Z) on the Serial Monitor
  Serial.print("Gyroscope Data (X, Y, Z): ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);
 
  // Display the accelerometer values (X, Y, Z) on the Serial Monitor
  Serial.print("Accelerometer Data (X, Y, Z): ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);
 
  delay(500); // Delay for readability (500ms)
}
