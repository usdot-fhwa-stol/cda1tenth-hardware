#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <SparkFunLSM6DSO.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

struct SensorData {
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float temperature = 0.0f;
    float right_rpm = 0.0f;
    float left_rpm = 0.0f;
    uint32_t last_update_ms = 0;
    bool data_valid = false;
    SemaphoreHandle_t mutex;
};

class SensorManager {
public:
    SensorManager();
    ~SensorManager();
    
    // Initialization
    bool initialize();
    void cleanup();
    
    // Data access
    SensorData getLatestData();
    bool isDataValid() const;
    
    // Update methods
    void update();
    
    // IMU specific methods
    bool isIMUReady() const;
    float getTemperature() const;
    
    // Motor data methods
    void setMotorRPM(float right_rpm, float left_rpm);
    
    // Task management
    void startSensorTask();
    void stopSensorTask();
    
private:
    // IMU
    LSM6DSO imu_;
    bool imu_initialized_;
    int cs_pin_;
    
    // Sensor data
    SensorData sensor_data_;
    
    // FreeRTOS task
    TaskHandle_t sensor_task_handle_;
    static void sensorTask(void* parameter);
    
    // Internal methods
    void readIMUData();
    void updateSensorData();
};

#endif // SENSOR_MANAGER_H
