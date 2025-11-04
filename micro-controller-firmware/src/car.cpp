#include "car.h"
#include <limits.h>
#include <math.h>

SteeringMotor::SteeringMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void SteeringMotor::begin()
{
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  driver.begin();
  driver.microsteps(MICROSTEPS);
  driver.RAMPMODE(0);

  driver.rms_current(200);
  driver.ihold(3);
  driver.irun(15);
  driver.iholddelay(5);

  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
  driver.toff(4);
  driver.blank_time(24);
  driver.TCOOLTHRS(0xFFFFF);
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.VDCMIN(0);

  driver.pwm_freq(1);
  driver.pwm_grad(1);
  driver.pwm_ofs(247);

  driver.a1(1000);
  driver.v1(1000);
  driver.AMAX(2000);
  driver.DMAX(2000);
  driver.VMAX(3000);
  driver.d1(1000);
  driver.VSTOP(10);

  lastCorrectionMicros = micros();
}

float SteeringMotor::normalizeAngle(float angle) const
{
  while (angle > 180.0f)
    angle -= DEGREES_PER_REVOLUTION;
  while (angle < -180.0f)
    angle += DEGREES_PER_REVOLUTION;
  return angle;
}

float SteeringMotor::getSteeringAngle() const
{
  int raw = analogRead(STEERING_SENSOR_PIN);
  float angle = ((float)raw / STEERING_SENSOR_MAX_VALUE) * DEGREES_PER_REVOLUTION;
  return angle;
}

void SteeringMotor::setTargetAngle(float angle)
{
  float newTargetAngle = normalizeAngle(-angle);
  if (fabsf(newTargetAngle - targetAngle) < 0.1f)
  {
    return;
  }

  targetAngle = newTargetAngle;

  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  float currentSteps = (currentAngle / DEGREES_PER_REVOLUTION) * stepsPerRev * STEERING_GEAR_RATIO;

  driver.XACTUAL((int32_t)currentSteps);
}

void SteeringMotor::setCarSpeed(float speed)
{
  carSpeed = speed;
}

void SteeringMotor::enableMotor(bool enable)
{
  motorEnabled = enable;
  if (enable)
  {
    driver.ihold(5);
    driver.irun(50);
  }
  else
  {
    driver.ihold(0);
    driver.irun(0);
  }
}

void SteeringMotor::setEncoderOffset(float offset)
{
  angleOffset = offset;
}

void SteeringMotor::updatePosition()
{
  uint32_t now = micros();

  bool shouldEnable = fabsf(carSpeed) > 0.1f;
  if (shouldEnable != motorEnabled)
  {
    enableMotor(shouldEnable);
  }

  if (!motorEnabled)
  {
    return;
  }

  float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  float error = normalizeAngle(targetAngle - currentAngle);

  if (fabsf(currentAngle - lastExternalAngle) < SMALL_MOVEMENT_THRESHOLD)
  {
    stallCounter++;
  }
  else
  {
    stallCounter = 0;
  }
  lastExternalAngle = currentAngle;

  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  int32_t currentSteps = (int32_t)((currentAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);
  int32_t targetSteps = (int32_t)((targetAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);

  if (stallCounter > STALL_DETECTION_COUNT)
  {
    driver.XACTUAL(currentSteps);
    stallCounter = 0;
  }

  if (fabsf(error) > STEERING_MAX_ALLOWED_ERROR && fabsf(carSpeed) > 0.1f)
  {
    float maxCorrectionSteps = 200.0f;
    int32_t correctionSteps = (int32_t)(error * stepsPerRev * STEERING_GEAR_RATIO / 360.0f);

    correctionSteps = (int32_t)(correctionSteps * 1.5f);

    if (correctionSteps > maxCorrectionSteps)
      correctionSteps = maxCorrectionSteps;
    else if (correctionSteps < -maxCorrectionSteps)
      correctionSteps = -maxCorrectionSteps;

    int32_t newTargetSteps = currentSteps + correctionSteps;
    driver.XTARGET(newTargetSteps);
  }
  else
  {
    driver.XTARGET(currentSteps);
  }
}

DriveMotor::DriveMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void DriveMotor::begin()
{
  pinMode(cs_pin, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.microsteps(MICROSTEPS);
  driver.RAMPMODE(2);
  driver.shaft(true);
  driver.X_ENC(0);

  driver.rms_current(600);
  driver.ihold(5);
  driver.irun(20);
  driver.iholddelay(5);

  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
  driver.toff(4);
  driver.blank_time(24);
  driver.TCOOLTHRS(0xFFFFF);
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.VDCMIN(0);

  driver.pwm_freq(1);
  driver.pwm_grad(1);
  driver.pwm_ofs(247);

  driver.a1(1000);
  driver.v1(1000);
  driver.AMAX(1000);
  driver.DMAX(1000);
  driver.d1(1000);
  driver.VSTOP(10);

  last_time = micros();
  last_enc = driver.X_ENC();
  current_rpm = 0.0f;
}

void resetDriver(TMC5160Stepper &drv, int ENN_PIN)
{
  digitalWrite(ENN_PIN, HIGH);
  delayMicroseconds(100);
  drv.GSTAT(0b111);
  digitalWrite(ENN_PIN, LOW);
}

void DriveMotor::setSpeed(float rpm)
{
  target_steps_per_sec = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
  target_rpm = rpm;
}

void DriveMotor::updateControlLoop()
{
  if (driver.GSTAT() & (1 << 2))
  {
    resetDriver(driver, EN_PIN);
  }

  int32_t current_enc = driver.X_ENC();
  uint32_t now = micros();
  uint32_t dt = now - last_time;
  int32_t delta_enc = current_enc - last_enc;

  if (dt > 0 && last_time > 0)
  {
    float measured_ticks_per_sec = (float)delta_enc * 1e6f / dt;
    float measured_steps_per_sec = measured_ticks_per_sec * ((float)(MOTOR_STEPS * MICROSTEPS) / (float)ENCODER_TICKS_PER_REVOLUTION);
    current_rpm = (measured_steps_per_sec / (float)(MOTOR_STEPS * MICROSTEPS)) * 60.0f;

    if (target_rpm < 0.0f && current_rpm > 0.0f)
    {
      current_rpm = -current_rpm;
    }
  }
  else if (target_steps_per_sec == 0)
  {
    current_rpm = 0.0f;
  }

  if (step_rate_cmd != target_steps_per_sec)
  {
    step_rate_cmd = target_steps_per_sec;
    if (step_rate_cmd < 0.0f)
      step_rate_cmd = 0.0f;

    driver.VMAX(step_rate_cmd);
    driver.shaft(target_rpm < 0);
  }

  last_enc = current_enc;
  last_time = now;
}

float DriveMotor::getCurrentRPM() const
{
  return current_rpm;
}

Car::Car(int rightCS, int leftCS, int steerCS)
    : rightMotor(rightCS), leftMotor(leftCS), steeringMotor(steerCS)
{
}

void Car::applyMotorSpeeds()
{
  float directionAwareSteeringAngle = (speed >= 0) ? steeringAngle : -steeringAngle;
  const float steeringAngleRad = directionAwareSteeringAngle * (M_PI / 180.0f);
  constexpr float minTurnAngle = 0.01f;

  if (fabsf(steeringAngleRad) < minTurnAngle)
  {
    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(-speed);
  }
  else
  {
    float turning_radius = wheelbase / tanf(fabsf(steeringAngleRad));
    float inner_radius = turning_radius - (trackWidth / 2.0f);
    float outer_radius = turning_radius + (trackWidth / 2.0f);
    float speed_ratio = inner_radius / outer_radius;

    if (steeringAngleRad > 0)
    {
      rightMotor.setSpeed(speed);
      leftMotor.setSpeed(-speed * speed_ratio);
    }
    else
    {
      rightMotor.setSpeed(speed * speed_ratio);
      leftMotor.setSpeed(-speed);
    }
  }
}

bool Car::isDriverHealthy()
{
  return (millis() - last_motor_update_) < 1000;
}

void Car::emergencyStop()
{
  speed = 0.0f;
  steeringAngle = 0.0f;
  rightMotor.setSpeed(0.0f);
  leftMotor.setSpeed(0.0f);
  steeringMotor.setTargetAngle(0.0f);
  last_motor_update_ = millis();
}

void Car::updateControlLoops()
{
  rightMotor.updateControlLoop();
  leftMotor.updateControlLoop();
  steeringMotor.updatePosition();
}

void Car::begin()
{
  rightMotor.begin();
  delay(100);
  leftMotor.begin();
  delay(100);
  steeringMotor.begin();
  delay(100);
}

void Car::setSteeringAngle(float angle)
{
  steeringAngle = angle;
  steeringMotor.setTargetAngle(angle);
  last_steering_update_ = millis();
}

void Car::setSpeed(float rpm, float wheelbase, float trackWidth)
{
  this->wheelbase = wheelbase;
  this->trackWidth = trackWidth;
  speed = rpm;

  steeringMotor.setCarSpeed(speed);
  applyMotorSpeeds();
  last_motor_update_ = millis();
}

float Car::getRightMotorRPM() const
{
  return rightMotor.getCurrentRPM();
}

float Car::getLeftMotorRPM() const
{
  return -leftMotor.getCurrentRPM();
}

float Car::getActualSteeringAngle() const
{
  float rawAngle = steeringMotor.getSteeringAngle();
  float actualAngle = steeringMotor.normalizeAngle(rawAngle - steeringMotor.angleOffset);
  return actualAngle;
}
