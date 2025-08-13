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

class SteeringMotor {
public:
  TMC5160Stepper driver;
  int cs_pin;
  float angleOffset = 160.0f;
  float targetAngle = 0.0f;
  int32_t targetSpeed = 0;
  int32_t lastAppliedSpeed = INT32_MIN;
  uint32_t lastApplyMicros = 0;
  uint8_t currentRunCurrent = 0xFF;
  uint8_t currentHoldCurrent = 0xFF;
  uint32_t lastCurrentUpdateMicros = 0;

  SteeringMotor(int cs);
  void begin();
  void setSpeed(int32_t speed);
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

  DriveMotor(int cs);
  void begin();
  void setSpeed(float rpm);
  void updateControlLoop();
  float getCurrentRPM();
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

private:
  SemaphoreHandle_t carMutex;
  void lock();
  void unlock();
};

#endif // CAR_H   