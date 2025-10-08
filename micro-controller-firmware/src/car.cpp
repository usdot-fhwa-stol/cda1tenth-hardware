#include "car.h"
#include <limits.h>
#include <math.h>

SteeringMotor::SteeringMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void SteeringMotor::begin() {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  driver.begin();
  driver.rms_current(300);
  driver.ihold(5);     // Hold torque when idle
  driver.irun(50);      // Run torque
  driver.iholddelay(5); // Delay before lowering to hold current
  driver.microsteps(MICROSTEPS);
  driver.en_pwm_mode(false);
  driver.pwm_autoscale(true);
  driver.toff(3);
  driver.blank_time(24);

  // Internal motion profile parameters
  driver.a1(500);
  driver.v1(500);
  driver.AMAX(5000);
  driver.DMAX(5000);
  driver.VMAX(8000);
  driver.d1(500);
  driver.VSTOP(10);

  driver.RAMPMODE(0); // Positioning mode
  lastCorrectionMicros = micros();
}


float SteeringMotor::normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= DEGREES_PER_REVOLUTION;
  while (angle < -180.0f) angle += DEGREES_PER_REVOLUTION;
  return angle;
}

float SteeringMotor::getSteeringAngle() {
  int raw = analogRead(STEERING_SENSOR_PIN);
  float angle = ((float)raw / STEERING_SENSOR_MAX_VALUE) * DEGREES_PER_REVOLUTION; // Map to 0 to 360 degrees
  return angle;
}

void SteeringMotor::setTargetAngle(float angle) {
  // Only update if the target has actually changed
  float newTargetAngle = normalizeAngle(-angle);
  if (fabsf(newTargetAngle - targetAngle) < 0.1f) {
    return; // Target hasn't changed significantly, skip update
  }
  
  targetAngle = newTargetAngle;
  
  // Simplified target setting - let the position update loop handle resync
  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  float targetSteps = (targetAngle / DEGREES_PER_REVOLUTION) * stepsPerRev * STEERING_GEAR_RATIO;
  
  // Set target for internal motion control
  driver.XTARGET((int32_t)targetSteps);
}

void SteeringMotor::updatePosition() {
  uint32_t now = micros();
  if (now - lastCorrectionMicros < STEERING_CORRECTION_INTERVAL) return;
  lastCorrectionMicros = now;

  // Read actual steering angle from sensor
  float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  float error = normalizeAngle(targetAngle - currentAngle);

  // // Stall detection: if angle hasn't changed much, increment counter
  if (fabsf(currentAngle - lastExternalAngle) < SMALL_MOVEMENT_THRESHOLD) {
    stallCounter++;
  } else {
    stallCounter = 0;
  }
  lastExternalAngle = currentAngle;

  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  int32_t actualSteps = (int32_t)((currentAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);

  if (stallCounter > STALL_DETECTION_COUNT) {
    driver.XACTUAL(actualSteps);
    stallCounter = 0;
  }

  // Normal drift correction if error exceeds threshold
  if (fabsf(error) > STEERING_MAX_ALLOWED_ERROR) {
    driver.XACTUAL(actualSteps);
  }
}

void SteeringMotor::applySpeed() {
  // Empty: internal logic controls speed automatically
}


DriveMotor::DriveMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void DriveMotor::begin() {
  pinMode(cs_pin, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  driver.begin();
  driver.shaft(true);
  driver.rms_current(1000);
  driver.microsteps(MICROSTEPS);
  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
  driver.TCOOLTHRS(0xFFFFF);
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.toff(3);
  driver.blank_time(24);
  driver.ihold(10);
  driver.irun(31);
  driver.iholddelay(5);
  driver.VDCMIN(0);
  driver.a1(1000);
  driver.v1(1000);
  driver.AMAX(1000);
  driver.DMAX(1000);
  driver.d1(1000);
  driver.VSTOP(10);
  driver.RAMPMODE(2);
  driver.X_ENC(0);
}

bool tmc5160_recover(TMC5160Stepper& drv, int ENN_PIN) {
  // 1) disable outputs
  digitalWrite(ENN_PIN, HIGH);
  drv.toff(0);
 
  // 2) clear faults
  drv.GSTAT(0b111);
 
  // 3) let VCP recharge
  delay(3);
 
  drv.irun(10);
  drv.toff(3);
  digitalWrite(ENN_PIN, LOW);
 
  delay(2);
 
  // 5) confirm no UV_CP
  uint8_t gstat = drv.GSTAT();

  drv.irun(31);
  drv.toff(10);
  
  return (gstat & (1<<2)) == 0;
}

// void DriveMotor::setSpeed(float rpm) {
//   driver.RAMPMODE(2); // Velocity mode
//   step_rate_cmd = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
//   target_steps_per_sec = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
//   driver.VMAX(step_rate_cmd);
//   driver.shaft(rpm < 0);
// }

void DriveMotor::setSpeed(float rpm) {
  target_steps_per_sec = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
  target_rpm = rpm;
}

void DriveMotor::updateControlLoop() {
  // MINIMAL motor control - just set the target speed directly
  // No complex control loops, no error calculations, no rate limiting
  
  // Simple fault check only
  if (driver.GSTAT() & (1 << 2)) { // Check for UV_CP fault
    if (!tmc5160_recover(driver, EN_PIN)) {
      return; // Recovery failed, do not proceed
    }
  }
  
  // Just set the motor speed directly - no control loop
  driver.VMAX(target_steps_per_sec);
  driver.shaft(target_rpm < 0);
  
  // Update timing for next call
  last_time = micros();
}

float DriveMotor::getCurrentRPM() {
  return current_rpm;
}

float DriveMotor::getCurrentRPMAtomic() const {
  return current_rpm;  // Direct access to volatile variable
}

Car::Car(int rightCS, int leftCS, int steerCS)
  : rightMotor(rightCS), leftMotor(leftCS), steeringMotor(steerCS) {
    carMutex = xSemaphoreCreateMutex();
  }

void Car::lock() {
  xSemaphoreTake(carMutex, portMAX_DELAY);
}

void Car::unlock() {
  xSemaphoreGive(carMutex);
}

bool Car::tryLock(uint32_t timeoutMs) {
  return xSemaphoreTake(carMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void Car::updateControlLoops() {
  // TEMPORARILY DISABLED - Test if motor control is causing freeze
  // rightMotor.setSpeed(speed);
  // leftMotor.setSpeed(-speed);
  
  // Update motor control loops - DISABLED FOR TESTING
  // rightMotor.updateControlLoop();
  // leftMotor.updateControlLoop();
  // steeringMotor.updatePosition();
}

void Car::begin() {
  rightMotor.begin();
  leftMotor.begin();
  steeringMotor.begin();
}

void Car::setSteeringAngle(float angle) {
  // MINIMAL steering control - no mutex, no complex logic
  steeringAngle = angle;
  steeringMotor.setTargetAngle(angle);
}

void Car::setSpeed(float rpm, float wheelbase, float trackWidth) {
  // Simplified setSpeed - no mutex to prevent deadlocks
  // Just store the values and let the control loop handle the rest
  speed = rpm;
  wheelbase = wheelbase;
  trackWidth = trackWidth;
  
  // The control loop will apply these values on the next update
  // This prevents mutex deadlocks and system freezes
}

float Car::getRightMotorRPM() {
  lock();
  float rightRPM = rightMotor.getCurrentRPM();
  unlock();
  return rightRPM;
}

float Car::getLeftMotorRPM() {
  lock();
  float leftRPM = leftMotor.getCurrentRPM();
  unlock();
  return leftRPM;
}

float Car::getRightMotorRPMAtomic() const {
  return rightMotor.getCurrentRPMAtomic();
}

float Car::getLeftMotorRPMAtomic() const {
  return leftMotor.getCurrentRPMAtomic();
}

bool Car::isMovingFastEnough() {
  // Check if either motor is moving fast enough to warrant steering hold
  float rightRPM = getRightMotorRPMAtomic();
  float leftRPM = getLeftMotorRPMAtomic();
  
  // Use the absolute value to check speed regardless of direction
  float avgSpeed = (fabsf(rightRPM) + fabsf(leftRPM)) / 2.0f;
  
  return avgSpeed >= STEERING_HOLD_DISABLE_SPEED_THRESHOLD;
}