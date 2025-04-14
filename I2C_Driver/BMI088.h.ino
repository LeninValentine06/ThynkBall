/**
 * @file BMI088.h
 * @brief Header file for BMI088 IMU sensor driver
 * @author Claude
 * @date April 2025
 */

#ifndef BMI088_H
#define BMI088_H

#include <Arduino.h>
#include <Wire.h>

// I2C Addresses
#define BMI088_ACC_ADDRESS              0x19
#define BMI088_GYRO_ADDRESS             0x69

// Accelerometer registers
#define BMI088_ACC_CHIP_ID              0x00    // Expected: 0x1E
#define BMI088_ACC_ERR_REG              0x02
#define BMI088_ACC_STATUS               0x03
#define BMI088_ACC_X_LSB                0x12
#define BMI088_ACC_X_MSB                0x13
#define BMI088_ACC_Y_LSB                0x14
#define BMI088_ACC_Y_MSB                0x15
#define BMI088_ACC_Z_LSB                0x16
#define BMI088_ACC_Z_MSB                0x17
#define BMI088_ACC_SENSORTIME_0         0x18
#define BMI088_ACC_SENSORTIME_1         0x19
#define BMI088_ACC_SENSORTIME_2         0x1A
#define BMI088_ACC_INT_STAT_1           0x1D
#define BMI088_ACC_TEMP_MSB             0x22
#define BMI088_ACC_TEMP_LSB             0x23
#define BMI088_ACC_FIFO_LENGTH_0        0x24
#define BMI088_ACC_FIFO_LENGTH_1        0x25
#define BMI088_ACC_FIFO_DATA            0x26
#define BMI088_ACC_RANGE                0x41    // G-range selection
#define BMI088_ACC_CONF                 0x40    // ODR and bandwidth
#define BMI088_ACC_INT1_IO_CONF         0x53
#define BMI088_ACC_INT2_IO_CONF         0x54
#define BMI088_ACC_INT1_MAP             0x56
#define BMI088_ACC_INT2_MAP             0x57
#define BMI088_ACC_PWR_CONF             0x7C    // Power configuration
#define BMI088_ACC_PWR_CTRL             0x7D    // Power control
#define BMI088_ACC_SOFTRESET            0x7E    // Soft reset register

// Gyroscope registers
#define BMI088_GYRO_CHIP_ID             0x00    // Expected: 0x0F
#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07
#define BMI088_GYRO_INT_STAT_1          0x0A
#define BMI088_GYRO_FIFO_STATUS         0x0E
#define BMI088_GYRO_RANGE               0x0F    // Angular rate range
#define BMI088_GYRO_BW                  0x10    // Bandwidth and ODR
#define BMI088_GYRO_LPM1                0x11    // Low power mode
#define BMI088_GYRO_SOFTRESET           0x14    // Soft reset register
#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18
#define BMI088_GYRO_FIFO_CONFIG         0x3E
#define BMI088_GYRO_FIFO_DATA           0x3F

// Accelerometer range values
typedef enum {
    BMI088_ACC_RANGE_3G = 0x00,  // +/- 3g
    BMI088_ACC_RANGE_6G = 0x01,  // +/- 6g
    BMI088_ACC_RANGE_12G = 0x02, // +/- 12g
    BMI088_ACC_RANGE_24G = 0x03  // +/- 24g
} bmi088_acc_range_t;

// Accelerometer ODR values
typedef enum {
    BMI088_ACC_ODR_12_5 = 0x05,  // 12.5 Hz
    BMI088_ACC_ODR_25 = 0x06,    // 25 Hz
    BMI088_ACC_ODR_50 = 0x07,    // 50 Hz
    BMI088_ACC_ODR_100 = 0x08,   // 100 Hz
    BMI088_ACC_ODR_200 = 0x09,   // 200 Hz
    BMI088_ACC_ODR_400 = 0x0A,   // 400 Hz
    BMI088_ACC_ODR_800 = 0x0B,   // 800 Hz
    BMI088_ACC_ODR_1600 = 0x0C   // 1600 Hz
} bmi088_acc_odr_t;

// Accelerometer bandwidth
typedef enum {
    BMI088_ACC_BW_OSR4 = 0x00,   // OSR4 mode
    BMI088_ACC_BW_OSR2 = 0x01,   // OSR2 mode
    BMI088_ACC_BW_NORMAL = 0x02  // Normal mode
} bmi088_acc_bw_t;

// Gyroscope range values
typedef enum {
    BMI088_GYRO_RANGE_2000 = 0x00,  // +/- 2000 degrees/second
    BMI088_GYRO_RANGE_1000 = 0x01,  // +/- 1000 degrees/second
    BMI088_GYRO_RANGE_500 = 0x02,   // +/- 500 degrees/second
    BMI088_GYRO_RANGE_250 = 0x03,   // +/- 250 degrees/second
    BMI088_GYRO_RANGE_125 = 0x04    // +/- 125 degrees/second
} bmi088_gyro_range_t;

// Gyroscope bandwidth and ODR
typedef enum {
    BMI088_GYRO_BW_ODR_2000_532 = 0x00,  // ODR: 2000Hz, Filter bandwidth: 532Hz
    BMI088_GYRO_BW_ODR_2000_230 = 0x01,  // ODR: 2000Hz, Filter bandwidth: 230Hz
    BMI088_GYRO_BW_ODR_1000_116 = 0x02,  // ODR: 1000Hz, Filter bandwidth: 116Hz
    BMI088_GYRO_BW_ODR_400_47 = 0x03,    // ODR: 400Hz, Filter bandwidth: 47Hz
    BMI088_GYRO_BW_ODR_200_23 = 0x04,    // ODR: 200Hz, Filter bandwidth: 23Hz
    BMI088_GYRO_BW_ODR_100_12 = 0x05,    // ODR: 100Hz, Filter bandwidth: 12Hz
    BMI088_GYRO_BW_ODR_200_64 = 0x06,    // ODR: 200Hz, Filter bandwidth: 64Hz
    BMI088_GYRO_BW_ODR_100_32 = 0x07     // ODR: 100Hz, Filter bandwidth: 32Hz
} bmi088_gyro_bw_t;

/**
 * @brief BMI088 sensor driver class for ESP32C6
 */
class BMI088 {
public:
    /**
     * @brief Constructor with optional custom I2C pins
     * @param wire Wire instance to use
     * @param sda SDA pin (optional)
     * @param scl SCL pin (optional)
     */
    BMI088(TwoWire &wire = Wire, int8_t sda = -1, int8_t scl = -1);
    
    /**
     * @brief Initialize the BMI088 sensor
     * @param accRange Accelerometer range
     * @param accODR Accelerometer output data rate
     * @param accBW Accelerometer bandwidth
     * @param gyroRange Gyroscope range
     * @param gyroBW Gyroscope bandwidth and ODR
     * @return true if initialization successful, false otherwise
     */
    bool begin(
        bmi088_acc_range_t accRange = BMI088_ACC_RANGE_12G,
        bmi088_acc_odr_t accODR = BMI088_ACC_ODR_400,
        bmi088_acc_bw_t accBW = BMI088_ACC_BW_NORMAL,
        bmi088_gyro_range_t gyroRange = BMI088_GYRO_RANGE_2000,
        bmi088_gyro_bw_t gyroBW = BMI088_GYRO_BW_ODR_400_47
    );
    
    /**
     * @brief Read accelerometer data
     * @param ax Pointer to store X-axis acceleration
     * @param ay Pointer to store Y-axis acceleration
     * @param az Pointer to store Z-axis acceleration
     * @param unit Unit for return values (true: g, false: raw values)
     * @return true if read successful, false otherwise
     */
    bool readAccelerometer(float *ax, float *ay, float *az, bool unit = true);
    
    /**
     * @brief Read gyroscope data
     * @param gx Pointer to store X-axis angular rate
     * @param gy Pointer to store Y-axis angular rate
     * @param gz Pointer to store Z-axis angular rate
     * @param unit Unit for return values (true: degrees/s, false: raw values)
     * @return true if read successful, false otherwise
     */
    bool readGyroscope(float *gx, float *gy, float *gz, bool unit = true);
    
    /**
     * @brief Read sensor temperature
     * @return Temperature in degrees Celsius
     */
    float readTemperature();
    
    /**
     * @brief Check if accelerometer is connected
     * @return true if connected, false otherwise
     */
    bool checkAccConnection();
    
    /**
     * @brief Check if gyroscope is connected
     * @return true if connected, false otherwise
     */
    bool checkGyroConnection();
    
    /**
     * @brief Set accelerometer range
     * @param range Range enum value
     * @return true if successful, false otherwise
     */
    bool setAccRange(bmi088_acc_range_t range);
    
    /**
     * @brief Set accelerometer ODR and bandwidth
     * @param odr Output data rate
     * @param bw Bandwidth
     * @return true if successful, false otherwise
     */
    bool setAccConfig(bmi088_acc_odr_t odr, bmi088_acc_bw_t bw);
    
    /**
     * @brief Set gyroscope range
     * @param range Range enum value
     * @return true if successful, false otherwise
     */
    bool setGyroRange(bmi088_gyro_range_t range);
    
    /**
     * @brief Set gyroscope bandwidth and ODR
     * @param bw Bandwidth and ODR enum value
     * @return true if successful, false otherwise
     */
    bool setGyroBandwidth(bmi088_gyro_bw_t bw);
    
    /**
     * @brief Soft reset accelerometer
     * @return true if successful, false otherwise
     */
    bool resetAcc();
    
    /**
     * @brief Soft reset gyroscope
     * @return true if successful, false otherwise
     */
    bool resetGyro();

private:
    TwoWire *_wire;             // Wire instance
    int8_t _sda;                // SDA pin
    int8_t _scl;                // SCL pin
    bmi088_acc_range_t _accRange;
    bmi088_gyro_range_t _gyroRange;
    float _accRangeFactor;      // Conversion factor for accelerometer
    float _gyroRangeFactor;     // Conversion factor for gyroscope
    
    /**
     * @brief Write data to accelerometer register
     * @param reg Register address
     * @param data Data to write
     * @return true if write successful, false otherwise
     */
    bool writeAccRegister(uint8_t reg, uint8_t data);
    
    /**
     * @brief Read data from accelerometer register
     * @param reg Register address
     * @param length Number of bytes to read
     * @param data Buffer to store read data
     * @return true if read successful, false otherwise
     */
    bool readAccRegister(uint8_t reg, uint8_t length, uint8_t *data);
    
    /**
     * @brief Write data to gyroscope register
     * @param reg Register address
     * @param data Data to write
     * @return true if write successful, false otherwise
     */
    bool writeGyroRegister(uint8_t reg, uint8_t data);
    
    /**
     * @brief Read data from gyroscope register
     * @param reg Register address
     * @param length Number of bytes to read
     * @param data Buffer to store read data
     * @return true if read successful, false otherwise
     */
    bool readGyroRegister(uint8_t reg, uint8_t length, uint8_t *data);
    
    /**
     * @brief Update accelerometer conversion factor based on range
     */
    void updateAccConversionFactor();
    
    /**
     * @brief Update gyroscope conversion factor based on range
     */
    void updateGyroConversionFactor();
};

#endif // BMI088_H