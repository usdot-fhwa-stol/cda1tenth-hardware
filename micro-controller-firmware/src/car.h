#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <limits.h>
#include "inter_core_comm.h"
#include "system_config.h"
#include "task_performance.h"
#include "mutex_wrapper.h"
#include "time_utils.h"
#include "validation_utils.h"
#include "error_handler.h"
#include "initializable.h"

// SPI pin definitions
#define CS_STEER    41
#define CS_RIGHT    39
#define CS_LEFT     40
#define MOSI_PIN    11
#define MISO_PIN    13
#define SCK_PIN     12
#define EN_PIN       4

// Stepper and driver parameters
#define R_SENSE 0.075f
#define MOTOR_STEPS 200
#define MICROSTEPS 16

// Steering sensor parameters
#define STEERING_SENSOR_PIN 18
#define STEERING_SENSOR_MAX_VALUE 4095.0f
#define DEGREES_PER_REVOLUTION 360.0f

// Encoder parameters
#define ENCODER_TICKS_PER_REVOLUTION 4000.0f

// Steering control parameters
#define STEERING_DEADBAND 0.5f
#define STEERING_KP 400.0f
#define STEERING_MAX_SPEED 12000.0f
#define STEERING_ERROR_THRESHOLD 5.0f
#define STEERING_CORRECTION_INTERVAL 50000  // 50 ms
#define STEERING_GEAR_RATIO (55.0f/12.0f) // ≈ 4.5833
#define STEERING_MAX_ALLOWED_ERROR 2.0f
#define STALL_DETECTION_COUNT 10 // ~500 ms
#define SMALL_MOVEMENT_THRESHOLD 0.5f // degrees

// Drive motor control parameters
#define DRIVE_ERROR_GAIN 0.1f
#define DRIVE_STALL_THRESHOLD 0.5f
#define DRIVE_STALL_REDUCTION 250
#define DRIVE_MAX_STALL_COUNT 5
#define MAX_STEP_ACCEL 200.0f

class SteeringMotor {
public:
  TMC5160Stepper driver;
  int cs_pin;
  float angleOffset = 187.5f;
  float targetAngle = 0.0f;
  int32_t targetSpeed = 0;
  int32_t lastAppliedSpeed = INT32_MIN;
  uint32_t lastCorrectionMicros = 0;
  float lastExternalAngle = 0.0f;
  int stallCounter = 0;
  
  SteeringMotor(int cs);
  void begin();
  void setSpeed(int32_t speed);
  void setTargetAngle(float angle);
  float getSteeringAngle();
  void updatePosition();
  void applySpeed();
  float normalizeAngle(float angle);
};

class DriveMotor {
public:
  TMC5160Stepper driver;
  int cs_pin;
  uint32_t step_rate_cmd = 0;
  uint32_t target_steps_per_sec = 0;
  int32_t last_enc = 0;
  uint32_t last_time = 0;
  int stall_counter = 0;
  float current_rpm = 0.0f;
  float target_rpm = 0.0f;

  DriveMotor(int cs);
  void begin();
  void setSpeed(float rpm);
  void updateControlLoop();
  float getCurrentRPM();
};

// =============================================================================
// MOTOR CONTROL TASK CLASS
// =============================================================================

class MotorControlTask : public TaskInitializable {
public:
    // Constructor and destructor
    MotorControlTask();
    ~MotorControlTask();
    
    // Initialization and cleanup
    bool initialize(DriveMotor* rightMotor, DriveMotor* leftMotor, SteeringMotor* steeringMotor,
                   ThreadSafeCarState* carState, const SystemConfig_t& config);
    void cleanup();
    
    // Task management
    bool startTask();
    void stopTask();
    bool isRunning();
    
    // Performance monitoring
    MotorControlPerformanceMetrics getPerformanceMetrics();
    void resetPerformanceMetrics();
    
    // Health monitoring
    bool isHealthy() const;
    uint32_t getErrorCount();
    
protected:
    // Virtual method implementations from TaskInitializable
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    virtual bool createTask() override;
    virtual void destroyTask() override;
    virtual void taskFunction() override;
    
private:
    // Motor references
    DriveMotor* rightMotor;
    DriveMotor* leftMotor;
    SteeringMotor* steeringMotor;
    ThreadSafeCarState* carState;
    
    // Task management
    TaskHandle_t taskHandle;
    
    // Timing control
    TickType_t lastWakeTime;
    const TickType_t taskPeriodTicks;
    
    // SPI protection
    SpiMutex spiMutex;
    
    // Performance monitoring
    MotorControlPerformanceMetrics performance;
    
    // System configuration
    SystemConfig_t config;
    
    // Private methods
    void executeControlLoop();
    void processMotorCommands();
    void updateMotorControlLoops();
    void reportMotorStatus();
    bool validateMotorCommand(const MotorCommand_t& cmd);
    bool isDeadlineMissed();
};

// =============================================================================
// MULTI-CORE COMPATIBLE CAR CLASS
// =============================================================================

class MultiCoreCar : public Initializable {
public:
  // Constructor and destructor
  MultiCoreCar(int rightCS, int leftCS, int steerCS);
  ~MultiCoreCar();
  
  // Initialization and cleanup
  bool initialize();
  void cleanup();
  
  // Status reporting
  bool getCurrentStatus(MotorStatus_t& status);
  bool isHealthy();
  
  // Configuration
  void setSystemConfig(const SystemConfig_t& config);
  
  // Performance monitoring
  MotorControlPerformanceMetrics getPerformanceMetrics();
  
protected:
  // Virtual method implementations from Initializable
  virtual bool doInitialize() override;
  virtual void doCleanup() override;
  virtual bool doHealthCheck() const override;
  
private:
  // Motor instances
  DriveMotor rightMotor;
  DriveMotor leftMotor;
  SteeringMotor steeringMotor;
  
  // Inter-core communication
  ThreadSafeCarState* carState;
  
  // Task management
  MotorControlTask motorControlTask;
  
  // SPI isolation mutex (ensures single-core SPI access)
  SpiMutex spiMutex;
  
  // System configuration
  SystemConfig_t systemConfig;
  
  // Private methods
  bool initializeMotors();
  void cleanupMotors();
};


#endif // CAR_H   