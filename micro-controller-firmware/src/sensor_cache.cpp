#include "sensor_cache.h"
#include "car.h"
#include "SparkFunLSM6DSO.h"
#include <math.h>

// Global sensor cache instance
SensorCache g_sensor_cache;

SensorCache::SensorCache() : car_instance(nullptr), imu_instance(nullptr), 
                             use_fallback_mode(false), last_successful_read(0) {
    cache_mutex = xSemaphoreCreateMutex();
    if (cache_mutex == NULL) {
        // Handle error - could log via debug system when available
    }
}

SensorCache::~SensorCache() {
    if (cache_mutex != NULL) {
        vSemaphoreDelete(cache_mutex);
    }
}

void SensorCache::initialize(Car* car, LSM6DSO* imu) {
    car_instance = car;
    imu_instance = imu;
    
    // Initialize fallback data with safe defaults
    fallback_data.gyro_z = 0.0f;
    fallback_data.accel_x = 0.0f;
    fallback_data.accel_y = 0.0f;
    fallback_data.right_rpm = 0.0f;
    fallback_data.left_rpm = 0.0f;
    fallback_data.steering_angle = 0.0f;
    fallback_data.valid = true;
    fallback_data.timestamp = millis();
}

void SensorCache::update() {
    if (cache_mutex == NULL || car_instance == NULL || imu_instance == NULL) {
        return;
    }
    
    uint32_t now = millis();
    uint32_t last_update = last_update_time.load();
    
    // Throttle updates to maintain consistent rate
    if (now - last_update < UPDATE_INTERVAL_MS) {
        return;
    }
    
    if (xSemaphoreTake(cache_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;  // Skip update if can't get mutex quickly
    }
    
    SensorData new_data;
    new_data.timestamp = now;
    bool read_success = true;
    
    // Read IMU data with error handling
    if (!readIMUData(new_data.gyro_z, new_data.accel_x, new_data.accel_y)) {
        // Use previous IMU data or fallback
        if (use_fallback_mode || !cached_data.valid) {
            new_data.gyro_z = fallback_data.gyro_z;
            new_data.accel_x = fallback_data.accel_x;
            new_data.accel_y = fallback_data.accel_y;
        } else {
            new_data.gyro_z = cached_data.gyro_z;
            new_data.accel_x = cached_data.accel_x;
            new_data.accel_y = cached_data.accel_y;
        }
        read_success = false;
    }
    
    // Read motor data with error handling
    if (!readMotorData(new_data.right_rpm, new_data.left_rpm)) {
        // Use previous motor data or fallback
        if (use_fallback_mode || !cached_data.valid) {
            new_data.right_rpm = fallback_data.right_rpm;
            new_data.left_rpm = fallback_data.left_rpm;
        } else {
            new_data.right_rpm = cached_data.right_rpm;
            new_data.left_rpm = cached_data.left_rpm;
        }
        read_success = false;
    }
    
    // Read steering data with error handling
    if (!readSteeringData(new_data.steering_angle)) {
        // Use previous steering data or fallback
        if (use_fallback_mode || !cached_data.valid) {
            new_data.steering_angle = fallback_data.steering_angle;
        } else {
            new_data.steering_angle = cached_data.steering_angle;
        }
        read_success = false;
    }
    
    // Validate the complete sensor data
    new_data.valid = validateSensorData(new_data);
    
    // Update cache
    cached_data = new_data;
    last_update_time.store(now);
    data_valid.store(new_data.valid);
    
    // Update fallback mode status
    if (read_success) {
        last_successful_read = now;
        if (use_fallback_mode && (now - last_successful_read < FALLBACK_TIMEOUT_MS)) {
            use_fallback_mode = false;  // Exit fallback mode
        }
        updateFallbackData();  // Update fallback with good data
    } else {
        if (!use_fallback_mode && (now - last_successful_read > FALLBACK_TIMEOUT_MS)) {
            use_fallback_mode = true;  // Enter fallback mode
        }
    }
    
    xSemaphoreGive(cache_mutex);
}

bool SensorCache::readIMUData(float& gyro_z, float& accel_x, float& accel_y) {
    if (imu_instance == NULL) return false;
    
    try {
        // Read gyroscope Z-axis (for odometry)
        gyro_z = imu_instance->readFloatGyroZ();
        
        // Read accelerometer X and Y (for additional odometry validation)
        accel_x = imu_instance->readFloatAccelX();
        accel_y = imu_instance->readFloatAccelY();
        
        // Basic sanity checks
        if (isnan(gyro_z) || isnan(accel_x) || isnan(accel_y)) {
            return false;
        }
        
        // Check for reasonable ranges (adjust based on your IMU specs)
        if (fabs(gyro_z) > 2000.0f || fabs(accel_x) > 20.0f || fabs(accel_y) > 20.0f) {
            return false;
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

bool SensorCache::readMotorData(float& right_rpm, float& left_rpm) {
    if (car_instance == NULL) return false;
    
    try {
        // Get RPM data from motor controllers
        right_rpm = car_instance->getRightMotorRPM();
        left_rpm = car_instance->getLeftMotorRPM();
        
        // Basic sanity checks
        if (isnan(right_rpm) || isnan(left_rpm)) {
            return false;
        }
        
        // Check for reasonable RPM ranges (adjust based on your motors)
        if (fabs(right_rpm) > 5000.0f || fabs(left_rpm) > 5000.0f) {
            return false;
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

bool SensorCache::readSteeringData(float& steering_angle) {
    if (car_instance == NULL) return false;
    
    try {
        steering_angle = car_instance->steeringMotor.getSteeringAngle();
        
        // Basic sanity checks
        if (isnan(steering_angle)) {
            return false;
        }
        
        // Normalize angle to reasonable range
        while (steering_angle > 180.0f) steering_angle -= 360.0f;
        while (steering_angle < -180.0f) steering_angle += 360.0f;
        
        return true;
    } catch (...) {
        return false;
    }
}

void SensorCache::updateFallbackData() {
    // Update fallback data with current good readings
    if (cached_data.valid) {
        fallback_data = cached_data;
        fallback_data.timestamp = millis();
    }
}

bool SensorCache::validateSensorData(const SensorData& data) {
    // Check timestamp is recent
    uint32_t now = millis();
    if (now - data.timestamp > MAX_DATA_AGE_MS) {
        return false;
    }
    
    // Check for NaN values
    if (isnan(data.gyro_z) || isnan(data.accel_x) || isnan(data.accel_y) ||
        isnan(data.right_rpm) || isnan(data.left_rpm) || isnan(data.steering_angle)) {
        return false;
    }
    
    // Additional validation could be added here
    // (e.g., consistency checks between sensors)
    
    return true;
}

SensorData SensorCache::getCachedData() const {
    if (cache_mutex == NULL) {
        return SensorData();  // Return invalid data
    }
    
    SensorData result;
    if (xSemaphoreTake(cache_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        result = cached_data;
        xSemaphoreGive(cache_mutex);
    }
    
    return result;
}

bool SensorCache::isDataFresh(uint32_t max_age_ms) const {
    uint32_t now = millis();
    uint32_t last_update = last_update_time.load();
    return (now - last_update) <= max_age_ms;
}

bool SensorCache::isDataValid() const {
    return data_valid.load();
}

float SensorCache::getGyroZ() const {
    SensorData data = getCachedData();
    return data.valid ? data.gyro_z : 0.0f;
}

float SensorCache::getRightRPM() const {
    SensorData data = getCachedData();
    return data.valid ? data.right_rpm : 0.0f;
}

float SensorCache::getLeftRPM() const {
    SensorData data = getCachedData();
    return data.valid ? data.left_rpm : 0.0f;
}

float SensorCache::getSteeringAngle() const {
    SensorData data = getCachedData();
    return data.valid ? data.steering_angle : 0.0f;
}

uint32_t SensorCache::getLastUpdateTime() const {
    return last_update_time.load();
}

bool SensorCache::isInFallbackMode() const {
    return use_fallback_mode;
}

void SensorCache::resetFallbackMode() {
    use_fallback_mode = false;
    last_successful_read = millis();
}

void SensorCache::forceUpdate() {
    last_update_time.store(0);  // Force next update() call to execute
    update();
}