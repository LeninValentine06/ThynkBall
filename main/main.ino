#include "BMI088.h"
#include <math.h>

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;
BMI088 bmi088( BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS );

// Madgwick filter variables
float sampleFreq = 20.0f;  // 20Hz sample rate (50ms delay)
float beta = 0.1f;         // 2 * proportional gain
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion

// Fast inverse square-root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Madgwick IMU algorithm update
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Function to convert quaternion to Euler angles
void getEulerAngles(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    *roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (abs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        *pitch = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

void setup(void) {
    Wire.begin(22, 23);
    Serial.begin(115200);

    while (!Serial);
    Serial.println("BMI088 with Integrated Madgwick AHRS Filter");
    Serial.println("ax(m/s²), ay(m/s²), az(m/s²), gx(rad/s), gy(rad/s), gz(rad/s), temp(°C), roll(°), pitch(°), yaw(°), q0, q1, q2, q3");

    while (1) {
        if (bmi088.isConnection()) {
            bmi088.initialize();
            Serial.println("BMI088 is connected");
            break;
        } else {
            Serial.println("BMI088 is not connected");
        }
        delay(2000);
    }
}

void loop(void) {
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();

    // Convert to standard SI units
    float ax_ms2 = (ax / 1000.0) * 9.80665;  // mg to m/s²
    float ay_ms2 = (ay / 1000.0) * 9.80665;
    float az_ms2 = (az / 1000.0) * 9.80665;
    
    float gx_rads = gx * (PI / 180.0);  // degrees/s to rad/s
    float gy_rads = gy * (PI / 180.0);
    float gz_rads = gz * (PI / 180.0);

    // Update Madgwick filter (IMU mode - no magnetometer)
    MadgwickAHRSupdateIMU(gx_rads, gy_rads, gz_rads, ax_ms2, ay_ms2, az_ms2);
    
    // Convert quaternion to Euler angles
    float roll, pitch, yaw;
    getEulerAngles(q0, q1, q2, q3, &roll, &pitch, &yaw);
    
    // Convert radians to degrees for display
    roll = roll * (180.0 / PI);
    pitch = pitch * (180.0 / PI);
    yaw = yaw * (180.0 / PI);

    // Output raw sensor data in SI units
    Serial.print(ax_ms2, 3);
    Serial.print(",");
    Serial.print(ay_ms2, 3);
    Serial.print(",");
    Serial.print(az_ms2, 3);
    Serial.print(",");
    Serial.print(gx_rads, 4);
    Serial.print(",");
    Serial.print(gy_rads, 4);
    Serial.print(",");
    Serial.print(gz_rads, 4);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    
    // Output orientation data
    Serial.print(roll, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.print(yaw, 2);
    Serial.print(",");
    
    // Output quaternion
    Serial.print(q0, 4);
    Serial.print(",");
    Serial.print(q1, 4);
    Serial.print(",");
    Serial.print(q2, 4);
    Serial.print(",");
    Serial.print(q3, 4);

    Serial.println();
    delay(50);  // 20Hz update rate
}