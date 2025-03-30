#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // SDA = 21, SCL = 22

    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected! Check wiring.");
        while (1);
    }
    Serial.println("MPU6050 Connected!");

    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  // Read sensor data

    Serial.print("Accel X: "); Serial.print(a.acceleration.x);
    Serial.print("\t Y: "); Serial.print(a.acceleration.y);
    Serial.print("\t Z: "); Serial.println(a.acceleration.z);

    Serial.print("Gyro X: "); Serial.print(g.gyro.x);
    Serial.print("\t Y: "); Serial.print(g.gyro.y);
    Serial.print("\t Z: "); Serial.println(g.gyro.z);

    Serial.print("Temperature: "); Serial.print(temp.temperature);
    Serial.println(" °C");

    Serial.println("--------------------");
    delay(500);
}
