#include "sensor_manager.h"
#include "car.h"

// External references
extern Car car;

SensorManager::SensorManager()
    : imu_initialized_(false), cs_pin_(CS_IMU)
{
    // Initialize sensor data - no mutex needed in single-threaded design
}

SensorManager::~SensorManager()
{
    cleanup();
}

bool SensorManager::initialize()
{
    // Initialize IMU
    imu_.beginSPI(cs_pin_);
    if (imu_.initialize(BASIC_SETTINGS))
    {
        imu_initialized_ = true;
    }
    else
    {
        imu_initialized_ = false;
        return false;
    }

    // Start sensor task
    startSensorTask();

    return true;
}

void SensorManager::cleanup()
{
    stopSensorTask();
    // No mutex cleanup needed in single-threaded design
}

SensorData SensorManager::getLatestData()
{
    // Direct access - no mutex needed in single-threaded design
    return sensor_data_;
}

bool SensorManager::isDataValid() const
{
    return sensor_data_.data_valid;
}

void SensorManager::update()
{
    // Simplified update - no separate task, everything in main loop
    static uint32_t last_imu_read = 0;
    uint32_t now = millis();

    // Read IMU at 50Hz to match control loop frequency
    if (imu_initialized_ && (now - last_imu_read >= 20))
    {
        readIMUData();
        last_imu_read = now;
    }

    // Always try to get motor data (car is always initialized in new approach)
    sensor_data_.right_rpm = car.getRightMotorRPM();
    sensor_data_.left_rpm = car.getLeftMotorRPM();

    updateSensorData();
}

bool SensorManager::isIMUReady() const
{
    return imu_initialized_;
}

float SensorManager::getTemperature() const
{
    return sensor_data_.temperature;
}

void SensorManager::readIMUData()
{
    if (!imu_initialized_)
        return;

    // Read IMU data with error checking
    float gyro_x = imu_.readFloatGyroX() * (M_PI / 180.0f);
    float gyro_y = imu_.readFloatGyroY() * (M_PI / 180.0f);
    float gyro_z = imu_.readFloatGyroZ() * (M_PI / 180.0f);

    float accel_x = imu_.readFloatAccelX();
    float accel_y = imu_.readFloatAccelY();
    float accel_z = imu_.readFloatAccelZ();

    float temperature = imu_.readTempF();

    // Check for valid data (not NaN or infinity)
    if (!isnan(gyro_x) && !isnan(gyro_y) && !isnan(gyro_z) &&
        !isnan(accel_x) && !isnan(accel_y) && !isnan(accel_z) &&
        !isnan(temperature))
    {
        // Update sensor data - no mutex needed in single-threaded design
        sensor_data_.gyro_x = gyro_x;
        sensor_data_.gyro_y = gyro_y;
        sensor_data_.gyro_z = gyro_z;
        sensor_data_.accel_x = accel_x;
        sensor_data_.accel_y = accel_y;
        sensor_data_.accel_z = accel_z;
        sensor_data_.temperature = temperature;
        sensor_data_.last_update_ms = millis();
        sensor_data_.data_valid = true;
    }
    else
    {
        // Mark data as invalid if we got bad readings
        sensor_data_.data_valid = false;
    }
}

void SensorManager::updateSensorData()
{
    // Direct update - no mutex needed in single-threaded design
    sensor_data_.last_update_ms = millis();
    sensor_data_.data_valid = true;
}
