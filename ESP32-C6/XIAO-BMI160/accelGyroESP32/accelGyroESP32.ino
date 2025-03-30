#include <Wire.h>
#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

void setup() {
    Serial.begin(115200);
    delay(100);

    if (bmi160.softReset() != BMI160_OK) {
        Serial.println("reset false");
        while (1);
    }

    if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
        Serial.println("init false");
        while (1);
    }
}

void loop() {
    int16_t accelGyro[6] = {0};

    // Fetch accelerometer and gyroscope data
    if (bmi160.getAccelGyroData(accelGyro) == 0) {
        float gx = accelGyro[0] * 3.14 / 180.0;  // Gyro X (rad/s)
        float gy = accelGyro[1] * 3.14 / 180.0;  // Gyro Y (rad/s)
        float gz = accelGyro[2] * 3.14 / 180.0;  // Gyro Z (rad/s)
        float ax = accelGyro[3] / 16384.0;       // Accel X (g)
        float ay = accelGyro[4] / 16384.0;       // Accel Y (g)
        float az = accelGyro[5] / 16384.0;       // Accel Z (g)

        // Send data in CSV format
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
    } else {
        Serial.println("err");
    }
    delay(50);  // Adjust delay for smooth visualization
}
