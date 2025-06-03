#include "BMI088.h"
#include <math.h>

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;
BMI088 bmi088(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS);

// WORLD COORDINATE SYSTEM DEFINITION:
// X-axis: Along the pitch (toward boundary)
// Y-axis: Across the pitch (square leg to point)  
// Z-axis: Vertically upward from ground
// Origin: Ball starting position

// IMPROVED: Increased sample rate and adaptive timestep
float sampleFreq = 100.0f;  // FIXED: Increased from 20Hz to 100Hz
float beta = 0.1f;         
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion

// Timing variables for actual timestep measurement
unsigned long prev_time_micros = 0;
float dt = 0.01f;  // Default timestep, will be updated with actual measurements

// OPTIMAL VELOCITY ESTIMATOR
struct OptimalVelocityEstimator {
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;
    float prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;
    float filtered_vx = 0.0f, filtered_vy = 0.0f, filtered_vz = 0.0f;
    bool first_integration = true;
};

// POSITION ESTIMATOR
struct PositionEstimator {
    float pos_x_world = 0.0f, pos_y_world = 0.0f, pos_z_world = 0.0f;
    float pos_x_body = 0.0f, pos_y_body = 0.0f, pos_z_body = 0.0f;
    float prev_vx_world = 0.0f, prev_vy_world = 0.0f, prev_vz_world = 0.0f;
    float vx_world = 0.0f, vy_world = 0.0f, vz_world = 0.0f;
    bool first_integration = true;
    float total_distance = 0.0f;
    float max_height = 0.0f;
    float min_height = 0.0f;
};

OptimalVelocityEstimator vel_est;
PositionEstimator pos_est;

// CALIBRATED SENSOR VALUES (keep your existing calibration)
float ax_bias = -0.117459f;
float ay_bias = -0.055255f;
float az_bias = 0.057719f;
float gx_bias = 0.000124f;
float gy_bias = -0.001741f;
float gz_bias = 0.000210f;

// IMPROVED: Tuned thresholds for cricket ball motion
float accel_threshold = 0.20f;        // Slightly increased for ball motion
float gyro_threshold = 0.30f;         // Increased for ball spinning motion
float accel_variance_threshold = 0.003f;
float gyro_variance_threshold = 0.015f;

// Zero-velocity detection
int zero_velocity_count = 0;        
int zero_velocity_threshold = 15;   // Increased for more stable detection
bool is_stationary = false;

// Detection buffers
const int DETECTION_WINDOW = 20;
float accel_buffer[DETECTION_WINDOW] = {0};
float gyro_buffer[DETECTION_WINDOW] = {0};
int buffer_index = 0;
bool buffer_initialized = false;

// Adaptive filter parameters
float light_filter_alpha = 0.15f;    
float heavy_filter_alpha = 0.03f;    

// Noise handling
float motion_noise_threshold = 0.015f;    // Slightly increased
float stationary_noise_threshold = 0.05f; 
float world_vel_threshold = 0.008f;       // Slightly increased

// Drift correction parameters
float stationary_decay_factor = 0.94f;    // Slightly stronger decay
float motion_drift_correction = 0.9995f;   
float position_decay = 0.996f;            

// PHYSICS VALIDATION PARAMETERS
float max_realistic_speed = 50.0f;        // 180 km/h max for cricket
float min_ground_level = -0.5f;           // Allow slight underground for noise
float max_reasonable_height = 100.0f;     // 100m max height

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

// Variance calculation helper
float calculateVariance(float* buffer, int size) {
    float mean = 0.0f;
    float variance = 0.0f;
    int actual_size = buffer_initialized ? DETECTION_WINDOW : buffer_index;
    
    if (actual_size < 2) return 999.0f;
    
    for(int i = 0; i < actual_size; i++) {
        mean += buffer[i];
    }
    mean /= actual_size;
    
    for(int i = 0; i < actual_size; i++) {
        float diff = buffer[i] - mean;
        variance += diff * diff;
    }
    variance /= actual_size;
    
    return variance;
}

// Apply adaptive noise threshold
float applyAdaptiveThreshold(float acceleration, bool moving) {
    float threshold = moving ? motion_noise_threshold : stationary_noise_threshold;
    return (abs(acceleration) < threshold) ? 0.0f : acceleration;
}

// PHYSICS VALIDATION FUNCTION - FIXED: Use pos_est instead of vel_est for world velocities
void validatePhysics() {
    // Check for unrealistic speeds - FIXED: Use pos_est.vx_world instead of vel_est.vx_world
    float speed = sqrt(pos_est.vx_world*pos_est.vx_world + 
                      pos_est.vy_world*pos_est.vy_world + 
                      pos_est.vz_world*pos_est.vz_world);
    
    if (speed > max_realistic_speed) {
        Serial.print("WARNING: Unrealistic speed: ");
        Serial.print(speed * 3.6f); // Convert to km/h
        Serial.println(" km/h");
        
        // Scale down velocities to maximum realistic - FIXED: Use pos_est
        float scale_factor = max_realistic_speed / speed;
        pos_est.vx_world *= scale_factor;
        pos_est.vy_world *= scale_factor;
        pos_est.vz_world *= scale_factor;
        vel_est.filtered_vx *= scale_factor;
        vel_est.filtered_vy *= scale_factor;
        vel_est.filtered_vz *= scale_factor;
    }
    
    // Check for underground position
    if (pos_est.pos_z_world < min_ground_level) {
        pos_est.pos_z_world = 0.0f;  // Reset to ground level
    }
    
    // Check for unrealistic height
    if (pos_est.pos_z_world > max_reasonable_height) {
        Serial.print("WARNING: Unrealistic height: ");
        Serial.print(pos_est.pos_z_world);
        Serial.println(" m");
        pos_est.pos_z_world = max_reasonable_height;
    }
}

// COMPLETE MADGWICK AHRS IMPLEMENTATION
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

    // Compute feedback only if accelerometer measurement valid
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
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
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

    // FIXED: Use actual timestep instead of fixed sample frequency
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

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
        *pitch = copysign(M_PI / 2, sinp);
    else
        *pitch = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

// Gravity compensation (unchanged - this was correct)
void compensateGravity(float ax_corrected, float ay_corrected, float az_corrected, 
                      float q0, float q1, float q2, float q3, 
                      float* ax_comp, float* ay_comp, float* az_comp) {
    // Transform world gravity [0, 0, -9.80665] to body frame
    float grav_x = 2.0f * (q1 * q3 - q0 * q2) * 9.80665f;
    float grav_y = 2.0f * (q2 * q3 + q0 * q1) * 9.80665f;
    float grav_z = (q0*q0 - q1*q1 - q2*q2 + q3*q3) * 9.80665f;
    
    *ax_comp = ax_corrected - grav_x;
    *ay_comp = ay_corrected - grav_y;
    *az_comp = az_corrected - grav_z;
    
    // ADDED: Validate gravity compensation
    float grav_total = sqrt(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    if (abs(grav_total - 9.80665f) > 1.0f) {
        Serial.print("WARNING: Gravity compensation error: ");
        Serial.println(grav_total);
    }
}

// Zero-velocity detection (unchanged - this was working well)
bool detectZeroVelocity(float ax_comp, float ay_comp, float az_comp, float gx, float gy, float gz) {
    float accel_mag = sqrt(ax_comp*ax_comp + ay_comp*ay_comp + az_comp*az_comp);
    float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);
    
    accel_buffer[buffer_index] = accel_mag;
    gyro_buffer[buffer_index] = gyro_mag;
    buffer_index = (buffer_index + 1) % DETECTION_WINDOW;
    
    if (buffer_index == 0) buffer_initialized = true;
    if (!buffer_initialized && buffer_index < 10) return false;
    
    float accel_var = calculateVariance(accel_buffer, DETECTION_WINDOW);
    float gyro_var = calculateVariance(gyro_buffer, DETECTION_WINDOW);
    
    bool magnitude_ok = (accel_mag < accel_threshold && gyro_mag < gyro_threshold);
    bool variance_ok = (accel_var < accel_variance_threshold && gyro_var < gyro_variance_threshold);
    
    if (magnitude_ok && variance_ok) {
        zero_velocity_count++;
        if (zero_velocity_count >= zero_velocity_threshold) {
            return true;
        }
    } else {
        zero_velocity_count = 0;
    }
    
    return false;
}

// OPTIMAL VELOCITY INTEGRATION METHOD (FIXED: Use actual timestep)
void integrateVelocityOptimal(float ax_comp, float ay_comp, float az_comp) {
    // Apply adaptive noise thresholding
    bool moving = !is_stationary;
    float ax_filtered = applyAdaptiveThreshold(ax_comp, moving);
    float ay_filtered = applyAdaptiveThreshold(ay_comp, moving);
    float az_filtered = applyAdaptiveThreshold(az_comp, moving);
    
    // FIXED: Trapezoidal integration using actual timestep
    if (vel_est.first_integration) {
        vel_est.vx += ax_filtered * dt;
        vel_est.vy += ay_filtered * dt;
        vel_est.vz += az_filtered * dt;
        vel_est.first_integration = false;
    } else {
        vel_est.vx += (vel_est.prev_ax + ax_filtered) * dt * 0.5f;
        vel_est.vy += (vel_est.prev_ay + ay_filtered) * dt * 0.5f;
        vel_est.vz += (vel_est.prev_az + az_filtered) * dt * 0.5f;
    }
    
    // Store current acceleration for next iteration
    vel_est.prev_ax = ax_filtered;
    vel_est.prev_ay = ay_filtered;
    vel_est.prev_az = az_filtered;
    
    // Apply adaptive filtering
    float current_alpha = is_stationary ? heavy_filter_alpha : light_filter_alpha;
    vel_est.filtered_vx = (1.0f - current_alpha) * vel_est.filtered_vx + current_alpha * vel_est.vx;
    vel_est.filtered_vy = (1.0f - current_alpha) * vel_est.filtered_vy + current_alpha * vel_est.vy;
    vel_est.filtered_vz = (1.0f - current_alpha) * vel_est.filtered_vz + current_alpha * vel_est.vz;
    
    // Apply drift correction
    if (is_stationary) {
        vel_est.vx *= stationary_decay_factor;
        vel_est.vy *= stationary_decay_factor;
        vel_est.vz *= stationary_decay_factor;
        
        vel_est.filtered_vx *= stationary_decay_factor;
        vel_est.filtered_vy *= stationary_decay_factor;
        vel_est.filtered_vz *= stationary_decay_factor;
    } else {
        vel_est.vx *= motion_drift_correction;
        vel_est.vy *= motion_drift_correction;
        vel_est.vz *= motion_drift_correction;
    }
    
    // Complete reset if strongly stationary
    if (is_stationary && zero_velocity_count > zero_velocity_threshold * 2) {
        vel_est.vx = 0.0f;
        vel_est.vy = 0.0f;
        vel_est.vz = 0.0f;
        vel_est.filtered_vx = 0.0f;
        vel_est.filtered_vy = 0.0f;
        vel_est.filtered_vz = 0.0f;
    }
}

// TRANSFORM VELOCITY FROM BODY FRAME TO WORLD FRAME (unchanged - this was correct)
void transformVelocityToWorld(float vx_body, float vy_body, float vz_body,
                              float q0, float q1, float q2, float q3,
                              float* vx_world, float* vy_world, float* vz_world) {
    
    // Optimized quaternion rotation for velocity transformation
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    
    // Rotation matrix elements
    float r11 = q0q0 + q1q1 - q2q2 - q3q3;
    float r12 = 2.0f * (q1q2 - q0q3);
    float r13 = 2.0f * (q1q3 + q0q2);
    
    float r21 = 2.0f * (q1q2 + q0q3);
    float r22 = q0q0 - q1q1 + q2q2 - q3q3;
    float r23 = 2.0f * (q2q3 - q0q1);
    
    float r31 = 2.0f * (q1q3 - q0q2);
    float r32 = 2.0f * (q2q3 + q0q1);
    float r33 = q0q0 - q1q1 - q2q2 + q3q3;
    
    // Transform velocity: v_world = R * v_body
    *vx_world = r11 * vx_body + r12 * vy_body + r13 * vz_body;
    *vy_world = r21 * vx_body + r22 * vy_body + r23 * vz_body;
    *vz_world = r31 * vx_body + r32 * vy_body + r33 * vz_body;
}

// OPTIMAL POSITION INTEGRATION METHOD (FIXED: Use actual timestep)
void integratePositionOptimal(float vx_body_filt, float vy_body_filt, float vz_body_filt,
                              float q0, float q1, float q2, float q3) {
    
    // Step 1: Transform filtered body velocity to world velocity
    transformVelocityToWorld(vx_body_filt, vy_body_filt, vz_body_filt, 
                            q0, q1, q2, q3, 
                            &pos_est.vx_world, &pos_est.vy_world, &pos_est.vz_world);
    
    // Step 2: Apply noise thresholding in world frame
    if (abs(pos_est.vx_world) < world_vel_threshold) pos_est.vx_world = 0.0f;
    if (abs(pos_est.vy_world) < world_vel_threshold) pos_est.vy_world = 0.0f;
    if (abs(pos_est.vz_world) < world_vel_threshold) pos_est.vz_world = 0.0f;
    
    // Step 3: FIXED - Integrate world velocity using actual timestep
    if (pos_est.first_integration) {
        pos_est.pos_x_world += pos_est.vx_world * dt;
        pos_est.pos_y_world += pos_est.vy_world * dt;
        pos_est.pos_z_world += pos_est.vz_world * dt;
        pos_est.first_integration = false;
    } else {
        pos_est.pos_x_world += (pos_est.prev_vx_world + pos_est.vx_world) * dt * 0.5f;
        pos_est.pos_y_world += (pos_est.prev_vy_world + pos_est.vy_world) * dt * 0.5f;
        pos_est.pos_z_world += (pos_est.prev_vz_world + pos_est.vz_world) * dt * 0.5f;
        
        // Calculate trajectory statistics
        float distance_step = sqrt(
            pow(pos_est.vx_world * dt, 2) + 
            pow(pos_est.vy_world * dt, 2) + 
            pow(pos_est.vz_world * dt, 2)
        );
        pos_est.total_distance += distance_step;
    }
    
    // Store current world velocity for next iteration
    pos_est.prev_vx_world = pos_est.vx_world;
    pos_est.prev_vy_world = pos_est.vy_world;
    pos_est.prev_vz_world = pos_est.vz_world;
    
    // Step 4: Track height statistics
    if (pos_est.pos_z_world > pos_est.max_height) {
        pos_est.max_height = pos_est.pos_z_world;
    }
    if (pos_est.pos_z_world < pos_est.min_height) {
        pos_est.min_height = pos_est.pos_z_world;
    }
    
    // Step 5: Apply drift correction when stationary
    if (is_stationary) {
        pos_est.pos_x_world *= position_decay;
        pos_est.pos_y_world *= position_decay;
        pos_est.pos_z_world *= position_decay;
        
        if (zero_velocity_count > zero_velocity_threshold * 3) {
            resetPositionEstimation();
        }
    }
    
    // Step 6: Maintain body frame position for comparison/debugging
    pos_est.pos_x_body += vx_body_filt * dt;
    pos_est.pos_y_body += vy_body_filt * dt;
    pos_est.pos_z_body += vz_body_filt * dt;
    
    if (is_stationary) {
        pos_est.pos_x_body *= position_decay;
        pos_est.pos_y_body *= position_decay;
        pos_est.pos_z_body *= position_decay;
    }
}

// RESET POSITION ESTIMATION (unchanged - this was correct)
void resetPositionEstimation() {
    pos_est.pos_x_world = 0.0f;
    pos_est.pos_y_world = 0.0f;
    pos_est.pos_z_world = 0.0f;
    
    pos_est.pos_x_body = 0.0f;
    pos_est.pos_y_body = 0.0f;
    pos_est.pos_z_body = 0.0f;
    
    pos_est.prev_vx_world = 0.0f;
    pos_est.prev_vy_world = 0.0f;
    pos_est.prev_vz_world = 0.0f;
    
    pos_est.vx_world = 0.0f;
    pos_est.vy_world = 0.0f;
    pos_est.vz_world = 0.0f;
    
    pos_est.first_integration = true;
    
    pos_est.total_distance = 0.0f;
    pos_est.max_height = 0.0f;
    pos_est.min_height = 0.0f;
}

void setup(void) {
    Wire.begin(22, 23);
    Serial.begin(115200);

    while (!Serial);
    
    while (1) {
        if (bmi088.isConnection()) {
            bmi088.initialize();
            
            // IMPROVED: Optimize BMI088 settings for cricket ball tracking
            bmi088.setAccScaleRange(RANGE_12G);        // Higher range for impacts
            bmi088.setAccOutputDataRate(ODR_200);      // 200Hz for accelerometer  
            bmi088.setGyroScaleRange(RANGE_1000);      // Good range for ball spin
            bmi088.setGyroOutputDataRate(ODR_200_BW_64); // 200Hz for gyroscope
            
            Serial.println("BMI088 Connected - IMPROVED CRICKET BALL TRACKING");
            Serial.println("Improvements: 100Hz sampling + Actual timesteps + Physics validation + Optimized settings");
            Serial.println("World Coordinates: X=Along pitch, Y=Across pitch, Z=Up from ground");
            Serial.println("timestamp,raw_acc_x,raw_acc_y,raw_acc_z,g_acc_x,g_acc_y,g_acc_z,raw_gx,raw_gy,raw_gz,temp,roll,pitch,yaw,q0,q1,q2,q3,vx,vy,vz,vx_filt,vy_filt,vz_filt,vx_world,vy_world,vz_world,pos_x_world,pos_y_world,pos_z_world,pos_x_body,pos_y_body,pos_z_body,total_dist,max_height,min_height,stationary,dt_actual");
            break;
        } else {
            Serial.println("BMI088 connection failed");
        }
        delay(2000);
    }
    
    // Initialize timing
    prev_time_micros = micros();
}

void loop(void) {
    // IMPROVED: Measure actual timestep
    unsigned long current_time_micros = micros();
    if (prev_time_micros > 0) {
        float measured_dt = (current_time_micros - prev_time_micros) / 1000000.0f;
        
        // Sanity check on timestep (between 5ms and 50ms)
        if (measured_dt > 0.005f && measured_dt < 0.05f) {
            dt = measured_dt;
        } else {
            dt = 0.01f; // Fallback to 10ms if measurement seems wrong
        }
    }
    prev_time_micros = current_time_micros;
    
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();

    // Convert to SI units and apply bias correction (CONFIRMED CORRECT)
    float ax_ms2 = (ax / 1000.0) * 9.80665;  // mg to m/s²
    float ay_ms2 = (ay / 1000.0) * 9.80665;
    float az_ms2 = (az / 1000.0) * 9.80665;
    
    float gx_rads = gx * (PI / 180.0) - gx_bias;  // deg/s to rad/s
    float gy_rads = gy * (PI / 180.0) - gy_bias;
    float gz_rads = gz * (PI / 180.0) - gz_bias;

    float ax_corrected = ax_ms2 - ax_bias;
    float ay_corrected = ay_ms2 - ay_bias;
    float az_corrected = az_ms2 - az_bias;

    // Update orientation using Madgwick filter (now with actual timestep)
    MadgwickAHRSupdateIMU(gx_rads, gy_rads, gz_rads, ax_corrected, ay_corrected, az_corrected);
    
    float roll, pitch, yaw;
    getEulerAngles(q0, q1, q2, q3, &roll, &pitch, &yaw);
    roll = roll * (180.0 / PI);
    pitch = pitch * (180.0 / PI);
    yaw = yaw * (180.0 / PI);

    // Gravity compensation
    float ax_comp, ay_comp, az_comp;
    compensateGravity(ax_corrected, ay_corrected, az_corrected, q0, q1, q2, q3, &ax_comp, &ay_comp, &az_comp);

    // Zero-velocity detection
    is_stationary = detectZeroVelocity(ax_comp, ay_comp, az_comp, gx_rads, gy_rads, gz_rads);
    
    // OPTIMAL VELOCITY INTEGRATION (now with actual timestep)
    integrateVelocityOptimal(ax_comp, ay_comp, az_comp);
    
    // OPTIMAL POSITION INTEGRATION (now with actual timestep)
    integratePositionOptimal(vel_est.filtered_vx, vel_est.filtered_vy, vel_est.filtered_vz, q0, q1, q2, q3);

    // ADDED: Physics validation
    validatePhysics();

    // COMPREHENSIVE CSV OUTPUT WITH POSITION DATA + ACTUAL TIMESTEP
    Serial.print(millis()); Serial.print(",");
    Serial.print(ax_ms2, 3); Serial.print(",");
    Serial.print(ay_ms2, 3); Serial.print(",");
    Serial.print(az_ms2, 3); Serial.print(",");
    Serial.print(ax_comp, 3); Serial.print(",");
    Serial.print(ay_comp, 3); Serial.print(",");
    Serial.print(az_comp, 3); Serial.print(",");
    Serial.print(gx_rads, 4); Serial.print(",");
    Serial.print(gy_rads, 4); Serial.print(",");
    Serial.print(gz_rads, 4); Serial.print(",");
    Serial.print(temp); Serial.print(",");
    Serial.print(roll, 2); Serial.print(",");
    Serial.print(pitch, 2); Serial.print(",");
    Serial.print(yaw, 2); Serial.print(",");
    Serial.print(q0, 4); Serial.print(",");
    Serial.print(q1, 4); Serial.print(",");
    Serial.print(q2, 4); Serial.print(",");
    Serial.print(q3, 4); Serial.print(",");
    
    // Velocity data (body frame)
    Serial.print(vel_est.vx, 4); Serial.print(",");
    Serial.print(vel_est.vy, 4); Serial.print(",");
    Serial.print(vel_est.vz, 4); Serial.print(",");
    Serial.print(vel_est.filtered_vx, 4); Serial.print(",");
    Serial.print(vel_est.filtered_vy, 4); Serial.print(",");
    Serial.print(vel_est.filtered_vz, 4); Serial.print(",");
    
    // Velocity data (world frame)
    Serial.print(pos_est.vx_world, 4); Serial.print(",");
    Serial.print(pos_est.vy_world, 4); Serial.print(",");
    Serial.print(pos_est.vz_world, 4); Serial.print(",");
    
    // Position data (world frame - PRIMARY OUTPUT)
    Serial.print(pos_est.pos_x_world, 4); Serial.print(",");
    Serial.print(pos_est.pos_y_world, 4); Serial.print(",");
    Serial.print(pos_est.pos_z_world, 4); Serial.print(",");
    
    // Position data (body frame - for comparison)
    Serial.print(pos_est.pos_x_body, 4); Serial.print(",");
    Serial.print(pos_est.pos_y_body, 4); Serial.print(",");
    Serial.print(pos_est.pos_z_body, 4); Serial.print(",");
    
    // Trajectory statistics
    Serial.print(pos_est.total_distance, 4); Serial.print(",");
    Serial.print(pos_est.max_height, 4); Serial.print(",");
    Serial.print(pos_est.min_height, 4); Serial.print(",");
    Serial.print(is_stationary ? 1 : 0); Serial.print(",");
    
    // ADDED: Actual timestep for monitoring
    Serial.println(dt * 1000.0f, 2); // dt in milliseconds
    
    // IMPROVED: 100Hz update rate instead of 20Hz
    delay(10);  // 10ms = 100Hz (was 50ms = 20Hz)
}