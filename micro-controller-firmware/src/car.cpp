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
  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;

  // Get current actual angle from external encoder
  float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);

  targetAngle = normalizeAngle(-angle);

  // Compute actual motor steps for current real position
  int32_t actualSteps = (int32_t)((currentAngle / DEGREES_PER_REVOLUTION) * stepsPerRev * STEERING_GEAR_RATIO);

  // Resync internal position to external encoder
  driver.XACTUAL(actualSteps);

  // Compute new target steps for desired angle
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
    // Full resync: align both actual and target to avoid fighting
    driver.XACTUAL(actualSteps);
    driver.XTARGET(actualSteps);
    stallCounter = 0;
    return;
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
  if (driver.GSTAT() & (1 << 2)) { // Check for UV_CP fault
    if (!tmc5160_recover(driver, EN_PIN)) {
      return; // Recovery failed, do not proceed
    }
  }
  int32_t current_enc = driver.X_ENC();
  uint32_t now = micros();
  uint32_t dt = now - last_time;
  int32_t delta_enc = current_enc - last_enc;
  
  // Avoid division by zero
  if (dt > 0) {
    float measured_ticks_per_sec = (float)delta_enc * 1e6f / dt;
    float measured_steps_per_sec = measured_ticks_per_sec * (MOTOR_STEPS * MICROSTEPS / ENCODER_TICKS_PER_REVOLUTION);
    measured_steps_per_sec = abs(measured_steps_per_sec);

    // Calculate current RPM
    current_rpm = (measured_steps_per_sec / (MOTOR_STEPS * MICROSTEPS)) * 60.0f;

    float error = target_steps_per_sec - measured_steps_per_sec;
    int32_t adjustment = (int32_t)(error * DRIVE_ERROR_GAIN);

    if (measured_steps_per_sec < (DRIVE_STALL_THRESHOLD * step_rate_cmd)) {
      stall_counter++;
    } else {
      stall_counter = 0;
    }

    if (stall_counter > DRIVE_MAX_STALL_COUNT) {
      step_rate_cmd -= DRIVE_STALL_REDUCTION * stall_counter;
      if ((int32_t)step_rate_cmd < 0) step_rate_cmd = 0;
    } else {
      step_rate_cmd += adjustment;
      if ((int32_t)step_rate_cmd < 0) step_rate_cmd = 0;
    }

    // Limit rate of change
    float max_step_change = MAX_STEP_ACCEL * (dt / 1e6f);  // steps/sec

    if (target_steps_per_sec > step_rate_cmd + max_step_change) {
      step_rate_cmd += max_step_change;
    } else if (target_steps_per_sec < step_rate_cmd - max_step_change) {
      step_rate_cmd -= max_step_change;
    } else {
      step_rate_cmd = target_steps_per_sec;
    }

    // Clamp to non-negative
    if (step_rate_cmd < 0.0f) step_rate_cmd = 0.0f;

    driver.VMAX(step_rate_cmd);
    driver.shaft(target_rpm < 0);
  }

  last_enc = current_enc;
  last_time = now;
}

float DriveMotor::getCurrentRPM() {
  return current_rpm;
}

float DriveMotor::getCurrentRPMAtomic() {
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
  // Use non-blocking lock to prevent deadlocks
  if (tryLock(5)) {  // 5ms timeout
    rightMotor.updateControlLoop();
    leftMotor.updateControlLoop();
    
    // Always update steering position to reach target
    steeringMotor.updatePosition();
    
    // Simplified position hold logic - only disable when completely stopped
    if (isMovingFastEnough()) {
      steeringMotor.setPositionHoldEnabled(true);
    } else {
      // Disable position hold when moving slowly or stopped
      // steeringMotor.setPositionHoldEnabled(false);
      steeringMotor.setPositionHoldEnabled(true);
    }
    
    unlock();
  }
  // If lock fails, skip this update cycle to prevent blocking
}

void Car::begin() {
  rightMotor.begin();
  leftMotor.begin();
  steeringMotor.begin();
}

void Car::setSteeringAngle(float angle) {
  if (tryLock(5)) {  // 5ms timeout
    steeringAngle = angle;
    steeringMotor.setTargetAngle(angle);
    unlock();
  }
  // If lock fails, skip this update to prevent blocking
}

void Car::setSpeed(float rpm, float wheelbase, float trackWidth) {
  if (tryLock(5)) {  // 5ms timeout
    speed = rpm;

    // steeringAngle is stored in degrees in this class; convert to radians for trig
    const float steeringAngleRad = steeringAngle * (M_PI / 180.0f);
    constexpr float minTurnAngle = 0.01f; // radians

    if (fabsf(steeringAngleRad) < minTurnAngle) {
      rightMotor.setSpeed(rpm);
      leftMotor.setSpeed(-rpm);
      unlock();
      return;
    }

    float R = wheelbase / tanf(steeringAngleRad);
    float R_L = R - (trackWidth / 2.0f);
    float R_R = R + (trackWidth / 2.0f);

    float v_L = rpm * (R_L / R);
    float v_R = rpm * (R_R / R);

    rightMotor.setSpeed(v_R);

    // left motor requires opposite rotation of right motor due to mirroring on car
    leftMotor.setSpeed(-v_L);
    unlock();
  }
  // If lock fails, skip this update to prevent blocking
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

float Car::getRightMotorRPMAtomic() {
  return rightMotor.getCurrentRPMAtomic();
}

float Car::getLeftMotorRPMAtomic() {
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