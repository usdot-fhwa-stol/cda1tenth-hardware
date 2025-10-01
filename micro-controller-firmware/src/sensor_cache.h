#ifndef SENSOR_CACHE_H
#define SENSOR_CACHE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>

// Forward declarations
class Car;
class LSM6DSO;

struct SensorData {
    // IMU data
    float gyro_z;
    float accel_x;
    float accel_y;
    
    // Motor encoder data
    float right_rpm;
    float left_rpm;
    
    // Steering data
    float steering_angle;
    
    // Metadata
    uint32_t timestamp;
    bool valid;
    
    SensorData() : gyro_z(0.0f), accel_x(0.0f), accel_y(0.0f), 
                   right_rpm(0.0f), left_rpm(0.0f), steering_angle(0.0f),
                   timestamp(0), valid(false) {}
};

class SensorCache {
private:
    static const uint32_t UPDATE_INTERVAL_MS = 20;  // 50Hz update rate
    static const uint32_t MAX_DATA_AGE_MS = 100;    // Data considered stale after 100ms
    static const uint32_t FALLBACK_TIMEOUT_MS = 500; // Switch to fallback after 500ms
    
    SensorData cached_data;
    SensorData fallback_data;
    std::atomic<uint32_t> last_update_time{0};
    std::atomic<bool> data_valid{false};
    SemaphoreHandle_t cache_mutex;
    
    // References to hardware interfaces
    Car* car_instance;
    LSM6DSO* imu_instance;
    
    // Fallback mechanisms
    bool use_fallback_mode;
    uint32_t last_successful_read;
    
    // Helper functions
    bool readIMUData(float& gyro_z, float& accel_x, float& accel_y);
    bool readMotorData(float& right_rpm, float& left_rpm);
    bool readSteeringData(float& steering_angle);
    void updateFallbackData();
    bool validateSensorData(const SensorData& data);

public:
    SensorCache();
    ~SensorCache();
    
    // Initialize with hardware references
    void initialize(Car* car, LSM6DSO* imu);
    
    // Main update function - call from dedicated thread
    void update();
    
    // Thread-safe data access
    SensorData getCachedData() const;
    bool isDataFresh(uint32_t max_age_ms = MAX_DATA_AGE_MS) const;
    bool isDataValid() const;
    
    // Specific sensor data getters with fallback
    float getGyroZ() const;
    float getRightRPM() const;
    float getLeftRPM() const;
    float getSteeringAngle() const;
    
    // Status and diagnostics
    uint32_t getLastUpdateTime() const;
    bool isInFallbackMode() const;
    void resetFallbackMode();
    
    // Force immediate update (use sparingly)
    void forceUpdate();
};

#endif // SENSOR_CACHE_H