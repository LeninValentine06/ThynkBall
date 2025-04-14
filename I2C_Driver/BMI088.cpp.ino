/**
 * @file BMI088.cpp
 * @brief Implementation file for BMI088 IMU sensor driver
 * @author Claude
 * @date April 2025
 */

#include "BMI088.h"

// BMI088 Constants
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

// Command values
#define BMI088_ACC_SOFT_RESET_CMD   0xB6
#define BMI088_GYRO_SOFT_RESET_CMD  0xB6
#define BMI088_ACC_PWR_CTRL_ENABLE  0x04
#define BMI088_ACC_PWR_CONF_ACTIVE  0x00

// Conversion constants
#define BMI088_TEMP_FACTOR          0.125f
#define BMI088_TEMP_OFFSET          23.0f

BMI088::BMI088(TwoWire &wire, int8_t sda, int8_t scl) {
    _wire = &wire;
    _sda = sda;
    _scl = scl;
    _accRange = BMI088_ACC_RANGE_12G;
    _gyroRange = BMI088_GYRO_RANGE_2000;
    updateAccConversionFactor();
    updateGyroConversionFactor();
}

bool BMI088::begin(
    bmi088_acc_range_t accRange,
    bmi088_acc_odr_t accODR,
    bmi088_acc_bw_t accBW,
    bmi088_gyro_range_t gyroRange,
    bmi088_gyro_bw_t gyroBW
) {
    // Initialize I2C with custom pins if specified
    if (_sda != -1 && _scl != -1) {
        _wire->begin(_sda, _scl);
    } else {
        _wire->begin();
    }
    
    // Step 1: Perform soft reset on both sensors
    if (!resetAcc() || !resetGyro()) {
        return false;
    }
    
    // Step 2: Wait for reset to complete
    delay(50);  // More than the required 2ms to be safe
    
    // Step 3: Configure power modes
    // Set accelerometer to active mode
    if (!writeAccRegister(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_CONF_ACTIVE)) {
        return false;
    }
    delay(5);
    
    // Enable accelerometer
    if (!writeAccRegister(BMI088_ACC_PWR_CTRL, BMI088_ACC_PWR_CTRL_ENABLE)) {
        return false;
    }
    delay(5);
    
    // Step 4: Set accelerometer range and config
    if (!setAccRange(accRange)) {
        return false;
    }
    
    if (!setAccConfig(accODR, accBW)) {
        return false;
    }
    
    // Step 5: Set gyroscope range and bandwidth
    if (!setGyroRange(gyroRange)) {
        return false;
    }
    
    if (!setGyroBandwidth(gyroBW)) {
        return false;
    }
    
    // Step 6: Verify sensor IDs
    if (!checkAccConnection() || !checkGyroConnection()) {
        return false;
    }
    
    return true;
}

bool BMI088::checkAccConnection() {
    uint8_t chipId;
    if (!readAccRegister(BMI088_ACC_CHIP_ID, 1, &chipId)) {
        return false;
    }
    return (chipId == BMI088_ACC_CHIP_ID_VALUE);
}

bool BMI088::checkGyroConnection() {
    uint8_t chipId;
    if (!readGyroRegister(BMI088_GYRO_CHIP_ID, 1, &chipId)) {
        return false;
    }
    return (chipId == BMI088_GYRO_CHIP_ID_VALUE);
}

bool BMI088::readAccelerometer(float *ax, float *ay, float *az, bool unit) {
    uint8_t data[6];
    
    if (!readAccRegister(BMI088_ACC_X_LSB, 6, data)) {
        return false;
    }
    
    // BMI088 accelerometer data is organized as 12-bit values in a 16-bit format
    // Extract raw values (combine MSB and LSB)
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]) >> 4;
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]) >> 4;
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]) >> 4;
    
    if (unit) {
        // Convert to g units using the appropriate range factor
        *ax = rawX * _accRangeFactor;
        *ay = rawY * _accRangeFactor;
        *az = rawZ * _accRangeFactor;
    } else {
        // Return raw values
        *ax = (float)rawX;
        *ay = (float)rawY;
        *az = (float)rawZ;
    }
    
    return true;
}

bool BMI088::readGyroscope(float *gx, float *gy, float *gz, bool unit) {
    uint8_t data[6];
    
    if (!readGyroRegister(BMI088_GYRO_RATE_X_LSB, 6, data)) {
        return false;
    }
    
    // Extract raw values (combine MSB and LSB)
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);
    
    if (unit) {
        // Convert to degrees per second using the appropriate range factor
        *gx = rawX * _gyroRangeFactor;
        *gy = rawY * _gyroRangeFactor;
        *gz = rawZ * _gyroRangeFactor;
    } else {
        // Return raw values
        *gx = (float)rawX;
        *gy = (float)rawY;
        *gz = (float)rawZ;
    }
    
    return true;
}

float BMI088::readTemperature() {
    uint8_t data[2];
    
    if (!readAccRegister(BMI088_ACC_TEMP_MSB, 2, data)) {
        return 0.0f;
    }
    
    // Extract raw temperature value
    int16_t rawTemp = (int16_t)((data[0] << 3) | (data[1] >> 5));
    
    // Convert to Celsius according to datasheet formula
    if (rawTemp > 1023) {
        rawTemp -= 2048;
    }
    
    float temp = (rawTemp * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
    return temp;
}

bool BMI088::setAccRange(bmi088_acc_range_t range) {
    if (!writeAccRegister(BMI088_ACC_RANGE, range)) {
        return false;
    }
    
    _accRange = range;
    updateAccConversionFactor();
    delay(1);
    return true;
}

bool BMI088::setAccConfig(bmi088_acc_odr_t odr, bmi088_acc_bw_t bw) {
    uint8_t config = (bw << 4) | odr;
    if (!writeAccRegister(BMI088_ACC_CONF, config)) {
        return false;
    }
    delay(1);
    return true;
}

bool BMI088::setGyroRange(bmi088_gyro_range_t range) {
    if (!writeGyroRegister(BMI088_GYRO_RANGE, range)) {
        return false;
    }
    
    _gyroRange = range;
    updateGyroConversionFactor();
    delay(1);
    return true;
}

bool BMI088::setGyroBandwidth(bmi088_gyro_bw_t bw) {
    if (!writeGyroRegister(BMI088_GYRO_BW, bw)) {
        return false;
    }
    delay(1);
    return true;
}

bool BMI088::resetAcc() {
    return writeAccRegister(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFT_RESET_CMD);
}

bool BMI088::resetGyro() {
    return writeGyroRegister(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFT_RESET_CMD);
}

bool BMI088::writeAccRegister(uint8_t reg, uint8_t data) {
    _wire->beginTransmission(BMI088_ACC_ADDRESS);
    _wire->write(reg);
    _wire->write(data);
    return (_wire->endTransmission() == 0);
}

bool BMI088::readAccRegister(uint8_t reg, uint8_t length, uint8_t *data) {
    _wire->beginTransmission(BMI088_ACC_ADDRESS);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) {
        return false;
    }
    
    _wire->requestFrom(BMI088_ACC_ADDRESS, length);
    uint8_t i = 0;
    while (_wire->available() && i < length) {
        data[i++] = _wire->read();
    }
    
    return (i == length);
}

bool BMI088::writeGyroRegister(uint8_t reg, uint8_t data) {
    _wire->beginTransmission(BMI088_GYRO_ADDRESS);
    _wire->write(reg);
    _wire->write(data);
    return (_wire->endTransmission() == 0);
}

bool BMI088::readGyroRegister(uint8_t reg, uint8_t length, uint8_t *data) {
    _wire->beginTransmission(BMI088_GYRO_ADDRESS);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) {
        return false;
    }
    
    _wire->requestFrom(BMI088_GYRO_ADDRESS, length);
    uint8_t i = 0;
    while (_wire->available() && i < length) {
        data[i++] = _wire->read();
    }
    
    return (i == length);
}

void BMI088::updateAccConversionFactor() {
    // Calculate the acceleration conversion factor based on range
    switch (_accRange) {
        case BMI088_ACC_RANGE_3G:
            _accRangeFactor = 3.0f / 32768.0f;
            break;
        case BMI088_ACC_RANGE_6G:
            _accRangeFactor = 6.0f / 32768.0f;
            break;
        case BMI088_ACC_RANGE_12G:
            _accRangeFactor = 12.0f / 32768.0f;
            break;
        case BMI088_ACC_RANGE_24G:
            _accRangeFactor = 24.0f / 32768.0f;
            break;
        default:
            _accRangeFactor = 12.0f / 32768.0f;  // Default to 12g
    }
}

void BMI088::updateGyroConversionFactor() {
    // Calculate the gyroscope conversion factor based on range
    switch (_gyroRange) {
        case BMI088_GYRO_RANGE_2000:
            _gyroRangeFactor = 2000.0f / 32768.0f;
            break;
        case BMI088_GYRO_RANGE_1000:
            _gyroRangeFactor = 1000.0f / 32768.0f;
            break;
        case BMI088_GYRO_RANGE_500:
            _gyroRangeFactor = 500.0f / 32768.0f;
            break;
        case BMI088_GYRO_RANGE_250:
            _gyroRangeFactor = 250.0f / 32768.0f;
            break;
        case BMI088_GYRO_RANGE_125:
            _gyroRangeFactor = 125.0f / 32768.0f;
            break;
        default:
            _gyroRangeFactor = 2000.0f / 32768.0f;  // Default to 2000 dps
    }
}