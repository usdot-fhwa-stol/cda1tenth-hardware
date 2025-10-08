#include "sensor_manager.h"
#include "car.h"

// External references
extern Car car;
extern bool car_initialized;

SensorManager::SensorManager() 
    : imu_initialized_(false)
    , cs_pin_(14)
    , sensor_task_handle_(NULL)
{
    // Initialize sensor data
    sensor_data_.mutex = xSemaphoreCreateMutex();
}

SensorManager::~SensorManager() {
    cleanup();
}

bool SensorManager::initialize() {
    // Initialize IMU
    imu_.beginSPI(cs_pin_);
    if (imu_.initialize(BASIC_SETTINGS)) {
        imu_initialized_ = true;
    } else {
        imu_initialized_ = false;
        return false;
    }
    
    // Start sensor task
    startSensorTask();
    
    return true;
}

void SensorManager::cleanup() {
    stopSensorTask();
    
    if (sensor_data_.mutex) {
        vSemaphoreDelete(sensor_data_.mutex);
        sensor_data_.mutex = NULL;
    }
}

SensorData SensorManager::getLatestData() {
    SensorData data;
    
    if (xSemaphoreTake(sensor_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data = sensor_data_;
        xSemaphoreGive(sensor_data_.mutex);
    }
    
    return data;
}

bool SensorManager::isDataValid() const {
    return sensor_data_.data_valid;
}

void SensorManager::update() {
    // Simplified update - no separate task, everything in main loop
    static uint32_t last_imu_read = 0;
    uint32_t now = millis();
    
    // Read IMU at 10Hz to avoid overwhelming the system
    if (imu_initialized_ && (now - last_imu_read >= 100)) {
        readIMUData();
        last_imu_read = now;
    }
    
    if (car_initialized) {
        sensor_data_.right_rpm = car.getRightMotorRPMAtomic();
        sensor_data_.left_rpm = car.getLeftMotorRPMAtomic();
    }
    
    updateSensorData();
}

bool SensorManager::isIMUReady() const {
    return imu_initialized_;
}

float SensorManager::getTemperature() const {
    return sensor_data_.temperature;
}

void SensorManager::setMotorRPM(float right_rpm, float left_rpm) {
    // Simplified - no mutex to avoid conflicts with micro-ROS
    sensor_data_.right_rpm = right_rpm;
    sensor_data_.left_rpm = left_rpm;
}

void SensorManager::startSensorTask() {
    // Disable separate sensor task to avoid thread conflicts with micro-ROS
    // All sensor reading will be done in main loop
    sensor_task_handle_ = NULL;
}

void SensorManager::stopSensorTask() {
    if (sensor_task_handle_ != NULL) {
        vTaskDelete(sensor_task_handle_);
        sensor_task_handle_ = NULL;
    }
}

// Sensor task removed to avoid thread conflicts with micro-ROS
// All sensor operations now happen in main loop

void SensorManager::readIMUData() {
    if (!imu_initialized_) return;
    
    // Read IMU data
    float gyro_x = imu_.readFloatGyroX() * (M_PI / 180.0f);
    float gyro_y = imu_.readFloatGyroY() * (M_PI / 180.0f);
    float gyro_z = imu_.readFloatGyroZ() * (M_PI / 180.0f);
    
    float accel_x = imu_.readFloatAccelX();
    float accel_y = imu_.readFloatAccelY();
    float accel_z = imu_.readFloatAccelZ();
    
    float temperature = imu_.readTempF();
    
    // Update sensor data with mutex protection
    if (xSemaphoreTake(sensor_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensor_data_.gyro_x = gyro_x;
        sensor_data_.gyro_y = gyro_y;
        sensor_data_.gyro_z = gyro_z;
        sensor_data_.accel_x = accel_x;
        sensor_data_.accel_y = accel_y;
        sensor_data_.accel_z = accel_z;
        sensor_data_.temperature = temperature;
        sensor_data_.last_update_ms = millis();
        sensor_data_.data_valid = true;
        xSemaphoreGive(sensor_data_.mutex);
    }
}

void SensorManager::updateSensorData() {
    if (xSemaphoreTake(sensor_data_.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensor_data_.last_update_ms = millis();
        sensor_data_.data_valid = true;
        xSemaphoreGive(sensor_data_.mutex);
    }
}
