#include <Arduino.h>
#include <Wire.h>

// Custom I2C pins for ESP32-C6FH4
#define SDA_PIN 22
#define SCL_PIN 23

// BMI088 I2C addresses
#define BMI088_ACCEL_ADDRESS 0x19
#define BMI088_GYRO_ADDRESS  0x69

// BMI088 Register addresses
// Accelerometer registers
#define BMI088_ACC_CHIP_ID            0x00  // 0x1E
#define BMI088_ACC_DATA_X_LSB         0x12
#define BMI088_ACC_DATA_Y_LSB         0x14
#define BMI088_ACC_DATA_Z_LSB         0x16
#define BMI088_ACC_TEMP_MSB           0x22
#define BMI088_ACC_TEMP_LSB           0x23
#define BMI088_ACC_CONF               0x40
#define BMI088_ACC_RANGE              0x41
#define BMI088_ACC_PWR_CONF           0x7C
#define BMI088_ACC_PWR_CTRL           0x7D
#define BMI088_ACC_SOFTRESET          0x7E

// Gyroscope registers
#define BMI088_GYRO_CHIP_ID           0x00  // 0x0F
#define BMI088_GYRO_RATE_X_LSB        0x02
#define BMI088_GYRO_RATE_Y_LSB        0x04
#define BMI088_GYRO_RATE_Z_LSB        0x06
#define BMI088_GYRO_RANGE             0x0F
#define BMI088_GYRO_BANDWIDTH         0x10
#define BMI088_GYRO_LPM1              0x11
#define BMI088_GYRO_SOFTRESET         0x14

// Constants
#define BMI088_ACC_CHIP_ID_VALUE      0x1E
#define BMI088_GYRO_CHIP_ID_VALUE     0x0F
#define BMI088_SOFT_RESET_CMD         0xB6

// Function prototypes
uint8_t readAccelRegister(uint8_t reg);
void writeAccelRegister(uint8_t reg, uint8_t value);
int16_t readAccelInt16(uint8_t reg);
uint8_t readGyroRegister(uint8_t reg);
void writeGyroRegister(uint8_t reg, uint8_t value);
int16_t readGyroInt16(uint8_t reg);
float readTemperature();

// Scaling factors
float accelScale = 0.0017948717949f; // For 6g range: 6g/2^15 * 9.80665 m/s²/LSB
float gyroScale = 0.015259f;         // For 500 dps range: 500/2^15 °/s/LSB

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("BMI088 Direct Register Access Test");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Check accelerometer chip ID
  uint8_t accelChipId = readAccelRegister(BMI088_ACC_CHIP_ID);
  Serial.print("Accelerometer Chip ID: 0x");
  Serial.println(accelChipId, HEX);
  if (accelChipId != BMI088_ACC_CHIP_ID_VALUE) {
    Serial.println("WARNING: Accelerometer ID doesn't match expected value!");
  }
  
  // Check gyroscope chip ID
  uint8_t gyroChipId = readGyroRegister(BMI088_GYRO_CHIP_ID);
  Serial.print("Gyroscope Chip ID: 0x");
  Serial.println(gyroChipId, HEX);
  if (gyroChipId != BMI088_GYRO_CHIP_ID_VALUE) {
    Serial.println("WARNING: Gyroscope ID doesn't match expected value!");
  }
  
  // Reset both sensors
  writeAccelRegister(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET_CMD);
  writeGyroRegister(BMI088_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD);
  delay(50);
  
  // Configure accelerometer
  writeAccelRegister(BMI088_ACC_PWR_CONF, 0x00);  // Disable power saving
  delay(5);
  writeAccelRegister(BMI088_ACC_PWR_CTRL, 0x04);  // Enable accelerometer
  delay(5);
  writeAccelRegister(BMI088_ACC_RANGE, 0x01);     // 6g range
  writeAccelRegister(BMI088_ACC_CONF, 0x8A);      // ODR = 100Hz, BW = normal
  
  // Configure gyroscope
  writeGyroRegister(BMI088_GYRO_RANGE, 0x02);      // 500 dps range
  writeGyroRegister(BMI088_GYRO_BANDWIDTH, 0x02);  // ODR = 1000Hz, Filter = 116Hz
  
  delay(100);
}

void loop() {
  // Read accelerometer data
  int16_t rawAccelX = readAccelInt16(BMI088_ACC_DATA_X_LSB);
  int16_t rawAccelY = readAccelInt16(BMI088_ACC_DATA_Y_LSB);
  int16_t rawAccelZ = readAccelInt16(BMI088_ACC_DATA_Z_LSB);
  
  // Read gyroscope data
  int16_t rawGyroX = readGyroInt16(BMI088_GYRO_RATE_X_LSB);
  int16_t rawGyroY = readGyroInt16(BMI088_GYRO_RATE_Y_LSB);
  int16_t rawGyroZ = readGyroInt16(BMI088_GYRO_RATE_Z_LSB);
  
  // Read temperature
  float temperature = readTemperature();
  
  // Convert accelerometer to m/s²
  float accelX = rawAccelX * accelScale;
  float accelY = rawAccelY * accelScale;
  float accelZ = rawAccelZ * accelScale;
  
  // Convert gyroscope to degrees per second
  float gyroX = rawGyroX * gyroScale;
  float gyroY = rawGyroY * gyroScale;
  float gyroZ = rawGyroZ * gyroScale;
  
  // Print all sensor data
  Serial.println("--- BMI088 Sensor Data ---");
  
  // Print raw values
  Serial.println("Raw Values:");
  Serial.print("  Accel: X=");
  Serial.print(rawAccelX);
  Serial.print(", Y=");
  Serial.print(rawAccelY);
  Serial.print(", Z=");
  Serial.println(rawAccelZ);
  
  Serial.print("  Gyro: X=");
  Serial.print(rawGyroX);
  Serial.print(", Y=");
  Serial.print(rawGyroY);
  Serial.print(", Z=");
  Serial.println(rawGyroZ);
  
  // Print converted values
  Serial.println("Converted Values:");
  Serial.print("  Accel (m/s²): X=");
  Serial.print(accelX, 4);
  Serial.print(", Y=");
  Serial.print(accelY, 4);
  Serial.print(", Z=");
  Serial.println(accelZ, 4);
  
  Serial.print("  Gyro (deg/s): X=");
  Serial.print(gyroX, 4);
  Serial.print(", Y=");
  Serial.print(gyroY, 4);
  Serial.print(", Z=");
  Serial.println(gyroZ, 4);
  
  Serial.print("  Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");
  
  Serial.println("------------------------");
  delay(500);
}

// Function for direct accelerometer register reading
uint8_t readAccelRegister(uint8_t reg) {
  Wire.beginTransmission(BMI088_ACCEL_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  // According to BMI088 datasheet, accelerometer needs special handling
  Wire.requestFrom(BMI088_ACCEL_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Write to accelerometer register
void writeAccelRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI088_ACCEL_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Special function to read 16-bit value with BMI088 accelerometer protocol
int16_t readAccelInt16(uint8_t reg) {
  uint8_t lsb, msb;
  
  Wire.beginTransmission(BMI088_ACCEL_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  // For BMI088 accelerometer, we need to read one extra byte
  Wire.requestFrom(BMI088_ACCEL_ADDRESS, (uint8_t)2);
  if (Wire.available() >= 2) {
    lsb = Wire.read();
    msb = Wire.read();
  }
  
  // Combine bytes into 16-bit value
  int16_t value = (msb << 8) | lsb;
  
  // The accelerometer data is 12-bit, right-justified in a 16-bit word
  // Apply sign extension for 12-bit values
  if (value & 0x0800) {
    value |= 0xF000;  // Sign extension for negative values
  }
  
  return value;
}

// Function for direct gyroscope register reading
uint8_t readGyroRegister(uint8_t reg) {
  Wire.beginTransmission(BMI088_GYRO_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(BMI088_GYRO_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Write to gyroscope register
void writeGyroRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI088_GYRO_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read 16-bit value from gyroscope
int16_t readGyroInt16(uint8_t reg) {
  uint8_t lsb, msb;
  
  Wire.beginTransmission(BMI088_GYRO_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(BMI088_GYRO_ADDRESS, (uint8_t)2);
  if (Wire.available() >= 2) {
    lsb = Wire.read();
    msb = Wire.read();
  }
  
  // Combine bytes into 16-bit value (gyro is full 16-bit, no sign extension needed)
  return (msb << 8) | lsb;
}

// Read temperature data
float readTemperature() {
  uint8_t msb, lsb;
  
  // Read temperature registers
  Wire.beginTransmission(BMI088_ACCEL_ADDRESS);
  Wire.write(BMI088_ACC_TEMP_MSB);
  Wire.endTransmission();
  
  Wire.requestFrom(BMI088_ACCEL_ADDRESS, (uint8_t)2);
  if (Wire.available() >= 2) {
    msb = Wire.read();
    lsb = Wire.read();
  }
  
  // Convert according to datasheet
  // Temperature data is stored as 2's complement 11-bit value
  // with the resolution of 0.125°C/LSB and a temperature offset of 23°C
  int16_t rawTemp = (int16_t)((msb << 3) | (lsb >> 5));
  
  // Apply sign extension for 11-bit values
  if (rawTemp & 0x0400) {
    rawTemp |= 0xF800; // Extend negative number
  }
  
  // Convert according to datasheet formula: (raw_value * 0.125°C) + 23°C
  return (rawTemp * 0.125f) + 23.0f;
}