#include "car.h"
#include <limits.h>
#include <math.h>

SteeringMotor::SteeringMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void SteeringMotor::begin() {
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  driver.begin();
  driver.rms_current(300);
  driver.ihold(2);
  driver.irun(20);
  driver.iholddelay(5);
  driver.microsteps(MICROSTEPS);
  driver.en_pwm_mode(false);
  driver.pwm_autoscale(true);
  driver.toff(3);
  driver.blank_time(24);

  driver.a1(500);
  driver.v1(500);
  driver.AMAX(5000);
  driver.DMAX(5000);
  driver.VMAX(8000);
  driver.d1(500);
  driver.VSTOP(10);

  driver.RAMPMODE(1);
}

void SteeringMotor::setTargetAngle(float angle) {
  targetAngle = angle;
  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  float targetSteps = (angle / DEGREES_PER_REVOLUTION) * stepsPerRev;
  driver.XTARGET((int32_t)targetSteps);
}

void SteeringMotor::setSpeed(int32_t speed) {
  targetSpeed = speed;
}

void SteeringMotor::applySpeed() {
  const uint32_t now = micros();
  
  // Only apply speed if it has changed or enough time has passed
  if (targetSpeed != lastAppliedSpeed || (now - lastApplyMicros) > 1000) {
    driver.VMAX(abs(targetSpeed));
    driver.shaft(targetSpeed < 0);
    lastAppliedSpeed = targetSpeed;
    lastApplyMicros = now;
  }
}

float SteeringMotor::getSteeringAngle() {
  int raw = analogRead(STEERING_SENSOR_PIN);
  float angle = ((float)raw / STEERING_SENSOR_MAX_VALUE) * DEGREES_PER_REVOLUTION; // Map to 0 to 360 degrees
  return angle;
}

float SteeringMotor::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= DEGREES_PER_REVOLUTION;
    while (angle < -180.0f) angle += DEGREES_PER_REVOLUTION;
    return angle;
}
 
void SteeringMotor::updatePosition() {
    // Normalize current steering angle relative to offset
    float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
 
    // Normalize target angle too, so they are both in the same range
    float normalizedTarget = normalizeAngle(-targetAngle);
 
    // Error calculation with rollover handling (-180° to 180°)
    float error = normalizeAngle(normalizedTarget - currentAngle);
 
    if (fabsf(error) < STEERING_DEADBAND) {
        setSpeed(0);
        // Reduce run current when within deadband to lower heat
        const uint32_t now = micros();
        if (now - lastCurrentUpdateMicros > STEERING_CURRENT_UPDATE_INTERVAL) {
            driver.irun(10);
            driver.ihold(3);
            lastCurrentUpdateMicros = now;
        }
        return;
    }
 
    float speed = STEERING_KP * error;
 
    // Clamp speed
    if (speed > STEERING_MAX_SPEED) speed = STEERING_MAX_SPEED;
    if (speed < -STEERING_MAX_SPEED) speed = -STEERING_MAX_SPEED;
 
    setSpeed((int32_t)speed);
 
    // Adjust motor current based on error magnitude
    const uint32_t now = micros();
    if (now - lastCurrentUpdateMicros > STEERING_CURRENT_UPDATE_INTERVAL) {
        if (fabsf(error) > STEERING_ERROR_THRESHOLD) {
            driver.irun(45);
            driver.ihold(6);
        } else {
            driver.irun(15);
            driver.ihold(4);
        }
        lastCurrentUpdateMicros = now;
    }
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

void DriveMotor::setSpeed(float rpm) {
  driver.RAMPMODE(2); // Velocity mode
  step_rate_cmd = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
  target_steps_per_sec = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
  driver.VMAX(step_rate_cmd);
  driver.shaft(rpm < 0);
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

    driver.VMAX(step_rate_cmd);
  }

  last_enc = current_enc;
  last_time = now;
}

float DriveMotor::getCurrentRPM() {
  return current_rpm;
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

void Car::updateControlLoops() {
  lock();
  rightMotor.updateControlLoop();
  leftMotor.updateControlLoop();
  steeringMotor.updatePosition();
  steeringMotor.applySpeed();
  unlock();
}

void Car::begin() {
  rightMotor.begin();
  leftMotor.begin();
  steeringMotor.begin();
}

void Car::setSteeringAngle(float angle) {
  lock();
  steeringAngle = angle;
  steeringMotor.setTargetAngle(angle);
  unlock();
}

void Car::setSpeed(float rpm, float wheelbase, float trackWidth) {
  lock();
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