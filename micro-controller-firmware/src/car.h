#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
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

// Steering control parameters
#define STEERING_CORRECTION_INTERVAL 10000  // 10 ms for more responsive steering
#define STEERING_GEAR_RATIO (55.0f/12.0f) // ≈ 4.5833
#define STEERING_MAX_ALLOWED_ERROR 5.0f
#define STALL_DETECTION_COUNT 5
#define SMALL_MOVEMENT_THRESHOLD 0.5f // degrees

// Drive motor control parameters

class SteeringMotor {
public:
  TMC5160Stepper driver;
  int cs_pin;
  float angleOffset = 187.5f;
  float targetAngle = 0.0f;
  uint32_t lastCorrectionMicros = 0;
  float lastExternalAngle = 0.0f;
  int stallCounter = 0;
  float carSpeed = 0.0f;  // Track car speed for steering control
  bool motorEnabled = true;  // Track if motor is enabled
  
  SteeringMotor(int cs);
  void begin();
  void setTargetAngle(float angle);
  void setCarSpeed(float speed);  // Method to update car speed
  void enableMotor(bool enable);  // Method to enable/disable motor
  float getSteeringAngle();
  void updatePosition();
  float normalizeAngle(float angle);
};

class DriveMotor {
public:
  TMC5160Stepper driver;
  int cs_pin;
  uint32_t target_steps_per_sec = 0;
  uint32_t last_time = 0;
  float target_rpm = 0.0f;
  
  // Control loop variables
  int32_t last_enc = 0;
  float step_rate_cmd = 0.0f;
  float current_rpm = 0.0f;
  
  // Non-blocking control state
  bool driver_ready = true;
  
  // Control constants
  static const int ENCODER_TICKS_PER_REVOLUTION = 4096;

  DriveMotor(int cs);
  void begin();
  void setSpeed(float rpm);
  void updateControlLoop();
  float getCurrentRPM() const;
};


class Car {
public:
  float speed = 0.0f;
  float steeringAngle = 0.0f;
  float wheelbase = 0.185f;  // Default wheelbase
  float trackWidth = 0.15f;  // Default track width
  DriveMotor rightMotor;
  DriveMotor leftMotor;
  SteeringMotor steeringMotor;

  Car(int rightCS, int leftCS, int steerCS);
  void updateControlLoops();
  void begin();
  void setSteeringAngle(float angle);
  void setSpeed(float rpm, float wheelbase, float trackWidth);
  float getRightMotorRPM() const;
  float getLeftMotorRPM() const;
  
  
  // Timeout and error handling
  bool isDriverHealthy();
  void emergencyStop();
  

private:
  // Timeout tracking
  uint32_t last_motor_update_ = 0;
  uint32_t last_steering_update_ = 0;
  bool driver_healthy_ = true;
  
  // Helper functions
  void applyMotorSpeeds();
};

#endif // CAR_H   