// BMI088.h
#ifndef BMI088_H
#define BMI088_H

#include <Arduino.h>
#include <Wire.h>

// BMI088 Accelerometer and Gyroscope I2C addresses
// The default addresses - these match what was found on your custom board
#define BMI088_ACCEL_ADDRESS          0x19  // Accelerometer address (SDO_A pin = VDDIO)
#define BMI088_GYRO_ADDRESS           0x69  // Gyroscope address (SDO_G pin = VDDIO)

// Accelerometer registers (from datasheet)
#define BMI088_ACC_CHIP_ID            0x00  // 0x1E
#define BMI088_ACC_ERR_REG            0x02
#define BMI088_ACC_STATUS             0x03
#define BMI088_ACC_DATA_X_LSB         0x12
#define BMI088_ACC_DATA_X_MSB         0x13
#define BMI088_ACC_DATA_Y_LSB         0x14
#define BMI088_ACC_DATA_Y_MSB         0x15
#define BMI088_ACC_DATA_Z_LSB         0x16
#define BMI088_ACC_DATA_Z_MSB         0x17
#define BMI088_ACC_SENSORTIME_0       0x18
#define BMI088_ACC_SENSORTIME_1       0x19
#define BMI088_ACC_SENSORTIME_2       0x1A
#define BMI088_ACC_INT_STAT_1         0x1D
#define BMI088_ACC_TEMP_MSB           0x22
#define BMI088_ACC_TEMP_LSB           0x23
#define BMI088_ACC_FIFO_LENGTH_0      0x24
#define BMI088_ACC_FIFO_LENGTH_1      0x25
#define BMI088_ACC_FIFO_DATA          0x26
#define BMI088_ACC_CONF               0x40
#define BMI088_ACC_RANGE              0x41
#define BMI088_ACC_FIFO_DOWNS         0x45
#define BMI088_ACC_FIFO_WTM_0         0x46
#define BMI088_ACC_FIFO_WTM_1         0x47
#define BMI088_ACC_FIFO_CONFIG_0      0x48
#define BMI088_ACC_FIFO_CONFIG_1      0x49
#define BMI088_ACC_INT1_IO_CONF       0x53
#define BMI088_ACC_INT2_IO_CONF       0x54
#define BMI088_ACC_INT_MAP_DATA       0x58
#define BMI088_ACC_SELF_TEST          0x6D
#define BMI088_ACC_PWR_CONF           0x7C
#define BMI088_ACC_PWR_CTRL           0x7D
#define BMI088_ACC_SOFTRESET          0x7E

// Gyroscope registers (from datasheet)
#define BMI088_GYRO_CHIP_ID           0x00  // 0x0F
#define BMI088_GYRO_RATE_X_LSB        0x02
#define BMI088_GYRO_RATE_X_MSB        0x03
#define BMI088_GYRO_RATE_Y_LSB        0x04
#define BMI088_GYRO_RATE_Y_MSB        0x05
#define BMI088_GYRO_RATE_Z_LSB        0x06
#define BMI088_GYRO_RATE_Z_MSB        0x07
#define BMI088_GYRO_INT_STAT_1        0x0A
#define BMI088_GYRO_FIFO_STATUS       0x0E
#define BMI088_GYRO_RANGE             0x0F
#define BMI088_GYRO_BANDWIDTH         0x10
#define BMI088_GYRO_LPM1              0x11
#define BMI088_GYRO_LPM2              0x12
#define BMI088_GYRO_RATE_HBW          0x13
#define BMI088_GYRO_SOFTRESET         0x14
#define BMI088_GYRO_INT_CTRL          0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP  0x18
#define BMI088_GYRO_FIFO_WM_EN        0x1E
#define BMI088_GYRO_FIFO_DATA         0x3E
#define BMI088_GYRO_SELF_TEST         0x3C

// Constants
#define BMI088_ACC_CHIP_ID_VALUE      0x1E
#define BMI088_GYRO_CHIP_ID_VALUE     0x0F

// Soft reset command
#define BMI088_SOFT_RESET_CMD         0xB6

class BMI088 {
public:
    // Accelerometer settings (from datasheet)
    enum AccelerometerRange {
        ACC_RANGE_3G  = 0x00,  // ±3g
        ACC_RANGE_6G  = 0x01,  // ±6g
        ACC_RANGE_12G = 0x02,  // ±12g
        ACC_RANGE_24G = 0x03   // ±24g
    };

    enum AccelerometerODR {
        ACC_ODR_12_5  = 0x05,  // 12.5 Hz
        ACC_ODR_25    = 0x06,  // 25 Hz
        ACC_ODR_50    = 0x07,  // 50 Hz
        ACC_ODR_100   = 0x08,  // 100 Hz
        ACC_ODR_200   = 0x09,  // 200 Hz
        ACC_ODR_400   = 0x0A,  // 400 Hz
        ACC_ODR_800   = 0x0B,  // 800 Hz
        ACC_ODR_1600  = 0x0C   // 1600 Hz
    };

    enum AccelerometerBandwidth {
        ACC_BW_OSR4   = 0x00,  // OSR4 mode
        ACC_BW_OSR2   = 0x01,  // OSR2 mode
        ACC_BW_NORMAL = 0x02   // Normal mode
    };

    // Gyroscope settings (from datasheet)
    enum GyroscopeRange {
        GYRO_RANGE_2000_DPS = 0x00,  // ±2000 °/s
        GYRO_RANGE_1000_DPS = 0x01,  // ±1000 °/s
        GYRO_RANGE_500_DPS  = 0x02,  // ±500 °/s
        GYRO_RANGE_250_DPS  = 0x03,  // ±250 °/s
        GYRO_RANGE_125_DPS  = 0x04   // ±125 °/s
    };

    enum GyroscopeBandwidth {
        GYRO_BW_532_HZ   = 0x00,  // ODR = 2000 Hz, filter bandwidth = 532 Hz
        GYRO_BW_230_HZ   = 0x01,  // ODR = 2000 Hz, filter bandwidth = 230 Hz
        GYRO_BW_116_HZ   = 0x02,  // ODR = 1000 Hz, filter bandwidth = 116 Hz
        GYRO_BW_47_HZ    = 0x03,  // ODR = 400 Hz, filter bandwidth = 47 Hz
        GYRO_BW_23_HZ    = 0x04,  // ODR = 200 Hz, filter bandwidth = 23 Hz
        GYRO_BW_12_HZ    = 0x05,  // ODR = 100 Hz, filter bandwidth = 12 Hz
        GYRO_BW_64_HZ    = 0x06,  // ODR = 200 Hz, filter bandwidth = 64 Hz
        GYRO_BW_32_HZ    = 0x07   // ODR = 100 Hz, filter bandwidth = 32 Hz
    };

    // Constructor with default I2C pins for ESP32-C6
    BMI088(TwoWire &wire = Wire, uint8_t accelAddress = BMI088_ACCEL_ADDRESS, 
           uint8_t gyroAddress = BMI088_GYRO_ADDRESS, int sdaPin = -1, int sclPin = -1);
    
    bool begin();
    
    // Configuration functions
    void setAccelerometerRange(AccelerometerRange range);
    void setAccelerometerODR(AccelerometerODR odr);
    void setAccelerometerBandwidth(AccelerometerBandwidth bandwidth);
    void setGyroscopeRange(GyroscopeRange range);
    void setGyroscopeBandwidth(GyroscopeBandwidth bandwidth);
    
    // Data reading functions
    void readAccelerometer(float &x, float &y, float &z);
    void readGyroscope(float &x, float &y, float &z);
    float readTemperature();
    
    // Power mode control
    void setAccelerometerPowerMode(bool enabled);
    void setGyroscopePowerMode(bool enabled);
    
    // Reset functions
    void resetAccelerometer();
    void resetGyroscope();
    
    // Chip ID verification
    bool verifyChipIds();

private:
    TwoWire &_wire;
    uint8_t _accelAddress;
    uint8_t _gyroAddress;
    float _accelScale;
    float _gyroScale;
    int _sdaPin;
    int _sclPin;
    
    // Helper functions
    uint8_t readAccelRegister(uint8_t reg);
    void writeAccelRegister(uint8_t reg, uint8_t value);
    uint8_t readGyroRegister(uint8_t reg);
    void writeGyroRegister(uint8_t reg, uint8_t value);
    
    void readAccelRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    void readGyroRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif // BMI088_H