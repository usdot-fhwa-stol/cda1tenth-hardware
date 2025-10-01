#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <limits.h>

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

// Steering hold disable parameters
#define STEERING_HOLD_DISABLE_SPEED_THRESHOLD 1.0f  // RPM threshold to disable steering hold

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
  void setPositionHoldEnabled(bool enabled);
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
  volatile float current_rpm = 0.0f;  // Made volatile for atomic access
  float target_rpm = 0.0f;

  DriveMotor(int cs);
  void begin();
  void setSpeed(float rpm);
  void updateControlLoop();
  float getCurrentRPM();
  float getCurrentRPMAtomic();  // Non-blocking atomic read
};

class Car {
public:
  float speed = 0.0f;
  float steeringAngle = 0.0f;
  DriveMotor rightMotor;
  DriveMotor leftMotor;
  SteeringMotor steeringMotor;

  Car(int rightCS, int leftCS, int steerCS);
  void updateControlLoops();
  void begin();
  void setSteeringAngle(float angle);
  void setSpeed(float rpm, float wheelbase, float trackWidth);
  float getRightMotorRPM();
  float getLeftMotorRPM();
  float getRightMotorRPMAtomic();  // Non-blocking atomic read
  float getLeftMotorRPMAtomic();   // Non-blocking atomic read
  bool isMovingFastEnough();       // Check if car is moving fast enough for steering hold

private:
  SemaphoreHandle_t carMutex;
  void lock();
  void unlock();
  bool tryLock(uint32_t timeoutMs = 10);  // Non-blocking lock with timeout
};

#endif // CAR_H   