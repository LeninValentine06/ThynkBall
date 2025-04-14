// BMI088.cpp
#include "BMI088.h"

BMI088::BMI088(TwoWire &wire, uint8_t accelAddress, uint8_t gyroAddress, int sdaPin, int sclPin) 
    : _wire(wire), _accelAddress(accelAddress), _gyroAddress(gyroAddress),
      _sdaPin(sdaPin), _sclPin(sclPin) {
    // Default scaling factors will be set during begin()
    _accelScale = 0.0f;
    _gyroScale = 0.0f;
}

bool BMI088::begin() {
    // Initialize I2C with custom pins if specified
    if (_sdaPin >= 0 && _sclPin >= 0) {
        _wire.begin(_sdaPin, _sclPin);
    } else {
        _wire.begin();
    }
    
    // Verify chip IDs
    if (!verifyChipIds()) {
        return false;
    }
    
    // Reset both sensors
    resetAccelerometer();
    resetGyroscope();
    
    delay(50); // Wait for reset to complete
    
    // Configure accelerometer
    writeAccelRegister(BMI088_ACC_PWR_CONF, 0x00); // Disable power saving mode
    delay(5);
    writeAccelRegister(BMI088_ACC_PWR_CTRL, 0x04); // Enable accelerometer
    delay(5);
    
    // Default configurations
    setAccelerometerRange(ACC_RANGE_12G);
    setAccelerometerODR(ACC_ODR_100);
    setAccelerometerBandwidth(ACC_BW_NORMAL);
    
    setGyroscopeRange(GYRO_RANGE_500_DPS);
    setGyroscopeBandwidth(GYRO_BW_116_HZ);
    
    return true;
}

bool BMI088::verifyChipIds() {
    // Check accelerometer chip ID
    uint8_t accelId = readAccelRegister(BMI088_ACC_CHIP_ID);
    if (accelId != BMI088_ACC_CHIP_ID_VALUE) {
        Serial.print("Accelerometer chip ID mismatch. Expected: 0x");
        Serial.print(BMI088_ACC_CHIP_ID_VALUE, HEX);
        Serial.print(", Got: 0x");
        Serial.println(accelId, HEX);
        return false;
    }
    
    // Check gyroscope chip ID
    uint8_t gyroId = readGyroRegister(BMI088_GYRO_CHIP_ID);
    if (gyroId != BMI088_GYRO_CHIP_ID_VALUE) {
        Serial.print("Gyroscope chip ID mismatch. Expected: 0x");
        Serial.print(BMI088_GYRO_CHIP_ID_VALUE, HEX);
        Serial.print(", Got: 0x");
        Serial.println(gyroId, HEX);
        return false;
    }
    
    return true;
}

void BMI088::setAccelerometerRange(AccelerometerRange range) {
    writeAccelRegister(BMI088_ACC_RANGE, (uint8_t)range);
    
    // Updated scaling factor calculations to match working implementation
    // These values directly calculate m/s² per LSB
    switch (range) {
        case ACC_RANGE_3G:
            _accelScale = 0.0008974358974f; // 3g/2^15 = 0.0000915 g/LSB * 9.80665 = 0.0008974358974 m/s²/LSB
            break;
        case ACC_RANGE_6G:
            _accelScale = 0.0017948717949f; // 6g/2^15 = 0.000183 g/LSB * 9.80665 = 0.0017948717949 m/s²/LSB
            break;
        case ACC_RANGE_12G:
            _accelScale = 0.0035897435897f; // 12g/2^15 = 0.000366 g/LSB * 9.80665 = 0.0035897435897 m/s²/LSB
            break;
        case ACC_RANGE_24G:
            _accelScale = 0.0071794871795f; // 24g/2^15 = 0.000732 g/LSB * 9.80665 = 0.0071794871795 m/s²/LSB
            break;
    }
}

void BMI088::setAccelerometerODR(AccelerometerODR odr) {
    uint8_t currentConf = readAccelRegister(BMI088_ACC_CONF);
    currentConf &= ~0xF0; // Clear ODR bits (bits 4-7)
    currentConf |= ((uint8_t)odr << 4); // Set new ODR
    writeAccelRegister(BMI088_ACC_CONF, currentConf);
}

void BMI088::setAccelerometerBandwidth(AccelerometerBandwidth bandwidth) {
    uint8_t currentConf = readAccelRegister(BMI088_ACC_CONF);
    currentConf &= ~0x0F; // Clear bandwidth bits (bits 0-3)
    currentConf |= (uint8_t)bandwidth; // Set new bandwidth
    writeAccelRegister(BMI088_ACC_CONF, currentConf);
}

void BMI088::setGyroscopeRange(GyroscopeRange range) {
    writeGyroRegister(BMI088_GYRO_RANGE, (uint8_t)range);
    
    // Update scaling factor based on datasheet values
    switch (range) {
        case GYRO_RANGE_2000_DPS:
            _gyroScale = 0.061037f; // 2000/2^15 = 0.061037 °/s/LSB
            break;
        case GYRO_RANGE_1000_DPS:
            _gyroScale = 0.030518f; // 1000/2^15 = 0.030518 °/s/LSB
            break;
        case GYRO_RANGE_500_DPS:
            _gyroScale = 0.015259f; // 500/2^15 = 0.015259 °/s/LSB
            break;
        case GYRO_RANGE_250_DPS:
            _gyroScale = 0.007629f; // 250/2^15 = 0.007629 °/s/LSB
            break;
        case GYRO_RANGE_125_DPS:
            _gyroScale = 0.003815f; // 125/2^15 = 0.003815 °/s/LSB
            break;
    }
}

void BMI088::setGyroscopeBandwidth(GyroscopeBandwidth bandwidth) {
    writeGyroRegister(BMI088_GYRO_BANDWIDTH, (uint8_t)bandwidth);
}

void BMI088::readAccelerometer(float &x, float &y, float &z) {
    uint8_t buffer[6];
    readAccelRegisters(BMI088_ACC_DATA_X_LSB, buffer, 6);
    
    // Combine data bytes to get the raw 16-bit values
    int16_t rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Apply sign extension for 12-bit values
    if (rawX & 0x0800) rawX |= 0xF000;
    if (rawY & 0x0800) rawY |= 0xF000;
    if (rawZ & 0x0800) rawZ |= 0xF000;
    
    // Convert to m/s² - the _accelScale already includes g conversion 
    x = rawX * _accelScale;
    y = rawY * _accelScale;
    z = rawZ * _accelScale;
}

void BMI088::readGyroscope(float &x, float &y, float &z) {
    uint8_t buffer[6];
    readGyroRegisters(BMI088_GYRO_RATE_X_LSB, buffer, 6);
    
    // Combine data bytes and convert to 16-bit signed integer
    // The gyroscope data is 16-bit, 2's complement
    int16_t rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to rad/s for standard scientific units
    // (degrees per second * π/180)
    x = rawX * _gyroScale * (PI / 180.0f);
    y = rawY * _gyroScale * (PI / 180.0f);
    z = rawZ * _gyroScale * (PI / 180.0f);
}

float BMI088::readTemperature() {
    uint8_t buffer[2];
    readAccelRegisters(BMI088_ACC_TEMP_MSB, buffer, 2);
    
    // According to the datasheet, temperature data is stored as 2's complement 11-bit value
    // with the resolution of 0.125°C/LSB and a temperature offset of 23°C
    int16_t rawTemp = (int16_t)((buffer[0] << 3) | (buffer[1] >> 5));
    
    // Apply sign extension for 11-bit values
    if (rawTemp & 0x0400) {
        rawTemp |= 0xF800; // Extend negative number
    }
    
    // Convert according to datasheet formula: (raw_value * 0.125°C) + 23°C
    return (rawTemp * 0.125f) + 23.0f;
}

void BMI088::setAccelerometerPowerMode(bool enabled) {
    if (enabled) {
        writeAccelRegister(BMI088_ACC_PWR_CONF, 0x00); // Disable power saving mode
        delay(5);
        writeAccelRegister(BMI088_ACC_PWR_CTRL, 0x04); // Enable accelerometer
    } else {
        writeAccelRegister(BMI088_ACC_PWR_CTRL, 0x00); // Disable accelerometer
        delay(5);
        writeAccelRegister(BMI088_ACC_PWR_CONF, 0x03); // Enable power save mode
    }
}

void BMI088::setGyroscopePowerMode(bool enabled) {
    if (enabled) {
        // Set normal mode
        writeGyroRegister(BMI088_GYRO_LPM1, 0x00); // Normal mode
    } else {
        // Set suspend mode
        writeGyroRegister(BMI088_GYRO_LPM1, 0x80); // Suspend mode (datasheet value: 0x80)
    }
}

void BMI088::resetAccelerometer() {
    writeAccelRegister(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET_CMD);
    delay(50); // Wait for reset to complete
}

void BMI088::resetGyroscope() {
    writeGyroRegister(BMI088_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD);
    delay(50); // Wait for reset to complete
}

uint8_t BMI088::readAccelRegister(uint8_t reg) {
    _wire.beginTransmission(_accelAddress);
    _wire.write(reg);
    _wire.endTransmission();
    
    // According to BMI088 datasheet, accelerometer needs special handling
    _wire.requestFrom(_accelAddress, (uint8_t)1);
    if (_wire.available()) {
        return _wire.read();
    }
    return 0;
}

void BMI088::writeAccelRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_accelAddress);
    _wire.write(reg);
    _wire.write(value);
    _wire.endTransmission();
}

uint8_t BMI088::readGyroRegister(uint8_t reg) {
    _wire.beginTransmission(_gyroAddress);
    _wire.write(reg);
    _wire.endTransmission();
    
    _wire.requestFrom(_gyroAddress, (uint8_t)1);
    if (_wire.available()) {
        return _wire.read();
    }
    return 0;
}

void BMI088::writeGyroRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_gyroAddress);
    _wire.write(reg);
    _wire.write(value);
    _wire.endTransmission();
}

void BMI088::readAccelRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    _wire.beginTransmission(_accelAddress);
    _wire.write(reg);
    _wire.endTransmission();
    
    // For BMI088 accelerometer, we need to read one extra byte
    _wire.requestFrom(_accelAddress, length);
    for (uint8_t i = 0; i < length && _wire.available(); i++) {
        buffer[i] = _wire.read();
    }
}

void BMI088::readGyroRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    _wire.beginTransmission(_gyroAddress);
    _wire.write(reg);
    _wire.endTransmission();
    
    _wire.requestFrom(_gyroAddress, length);
    for (uint8_t i = 0; i < length && _wire.available(); i++) {
        buffer[i] = _wire.read();
    }
}