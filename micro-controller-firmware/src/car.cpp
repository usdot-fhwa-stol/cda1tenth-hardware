#include "car.h"
#include <limits.h>
#include <math.h>

SteeringMotor::SteeringMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void SteeringMotor::begin()
{
  // Pin setup
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);

  // Driver initialization
  driver.begin();
  driver.microsteps(MICROSTEPS);
  driver.RAMPMODE(0); // Positioning mode

  // Current settings
  driver.rms_current(300);
  driver.ihold(5);
  driver.irun(50);
  driver.iholddelay(5);

  // PWM settings
  driver.en_pwm_mode(false);
  driver.pwm_autoscale(true);
  driver.toff(3);
  driver.blank_time(24);

  // Motion profile
  driver.a1(500);
  driver.v1(500);
  driver.AMAX(5000);
  driver.DMAX(5000);
  driver.VMAX(8000);
  driver.d1(500);
  driver.VSTOP(10);

  lastCorrectionMicros = micros();
}

float SteeringMotor::normalizeAngle(float angle)
{
  while (angle > 180.0f)
    angle -= DEGREES_PER_REVOLUTION;
  while (angle < -180.0f)
    angle += DEGREES_PER_REVOLUTION;
  return angle;
}

float SteeringMotor::getSteeringAngle()
{
  int raw = analogRead(STEERING_SENSOR_PIN);
  float angle = ((float)raw / STEERING_SENSOR_MAX_VALUE) * DEGREES_PER_REVOLUTION; // Map to 0 to 360 degrees
  return angle;
}

void SteeringMotor::setTargetAngle(float angle)
{
  // Only update if the target has actually changed
  float newTargetAngle = normalizeAngle(-angle);
  if (fabsf(newTargetAngle - targetAngle) < 0.1f)
  {
    return; // Target hasn't changed significantly, skip update
  }

  targetAngle = newTargetAngle;

  // Simplified target setting - let the position update loop handle resync
  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  float targetSteps = (targetAngle / DEGREES_PER_REVOLUTION) * stepsPerRev * STEERING_GEAR_RATIO;

  // Set target for internal motion control
  driver.XTARGET((int32_t)targetSteps);
}

void SteeringMotor::updatePosition()
{
  uint32_t now = micros();
  if (now - lastCorrectionMicros < STEERING_CORRECTION_INTERVAL)
    return;
  lastCorrectionMicros = now;

  // Read actual steering angle from sensor
  float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  float error = normalizeAngle(targetAngle - currentAngle);

  // // Stall detection: if angle hasn't changed much, increment counter
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
  int32_t actualSteps = (int32_t)((currentAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);

  if (stallCounter > STALL_DETECTION_COUNT)
  {
    driver.XACTUAL(actualSteps);
    stallCounter = 0;
  }

  // Normal drift correction if error exceeds threshold
  if (fabsf(error) > STEERING_MAX_ALLOWED_ERROR)
  {
    driver.XACTUAL(actualSteps);
  }
}

DriveMotor::DriveMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void DriveMotor::begin()
{
  // Pin setup
  pinMode(cs_pin, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  // Driver initialization
  driver.begin();
  driver.microsteps(MICROSTEPS);
  driver.RAMPMODE(2); // Velocity mode
  driver.shaft(true);
  driver.X_ENC(0);

  // Current settings
  driver.rms_current(1000);
  driver.ihold(10);
  driver.irun(31);
  driver.iholddelay(5);

  // PWM and stealthChop settings
  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
  driver.toff(3);
  driver.blank_time(24);
  driver.TCOOLTHRS(0xFFFFF);
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.VDCMIN(0);

  // Motion profile
  driver.a1(1000);
  driver.v1(1000);
  driver.AMAX(1000);
  driver.DMAX(1000);
  driver.d1(1000);
  driver.VSTOP(10);

  // Initialize RPM calculation variables
  last_time = micros();
  last_enc = driver.X_ENC();
  current_rpm = 0.0f;
}

// Simple driver reset function
void resetDriver(TMC5160Stepper &drv, int ENN_PIN)
{
  digitalWrite(ENN_PIN, HIGH);
  delayMicroseconds(100);
  drv.GSTAT(0b111); // Clear faults
  digitalWrite(ENN_PIN, LOW);
}

void DriveMotor::setSpeed(float rpm)
{
  target_steps_per_sec = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
  target_rpm = rpm;
}

void DriveMotor::updateControlLoop()
{
  // Simple fault check and reset
  if (driver.GSTAT() & (1 << 2))
  {
    resetDriver(driver, EN_PIN);
  }

  int32_t current_enc = driver.X_ENC();
  uint32_t now = micros();
  uint32_t dt = now - last_time;
  int32_t delta_enc = current_enc - last_enc;

  // Calculate RPM if we have valid timing data
  if (dt > 0 && last_time > 0)
  {
    float measured_ticks_per_sec = (float)delta_enc * 1e6f / dt;
    float measured_steps_per_sec = measured_ticks_per_sec * ((float)(MOTOR_STEPS * MICROSTEPS) / (float)ENCODER_TICKS_PER_REVOLUTION);
    current_rpm = (measured_steps_per_sec / (float)(MOTOR_STEPS * MICROSTEPS)) * 60.0f;

    // Apply direction
    if (target_rpm < 0.0f && current_rpm > 0.0f)
    {
      current_rpm = -current_rpm;
    }
  }
  else if (target_steps_per_sec == 0)
  {
    current_rpm = 0.0f;
  }

  // Update motor speed if changed
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
  // Software differential for front-wheel steering car
  // When turning, the inner wheel should slow down and outer wheel should speed up
  const float steeringAngleRad = steeringAngle * (M_PI / 180.0f);
  constexpr float minTurnAngle = 0.01f; // radians

  if (fabsf(steeringAngleRad) < minTurnAngle)
  {
    // Straight line - both motors same speed
    rightMotor.setSpeed(speed);
    leftMotor.setSpeed(-speed);
  }
  else
  {
    // Calculate differential speeds based on turning radius
    // For a turn, the inner wheel travels a shorter distance than the outer wheel
    float turning_radius = wheelbase / tanf(fabsf(steeringAngleRad));

    // Calculate the radius for each rear wheel
    float inner_radius = turning_radius - (trackWidth / 2.0f);
    float outer_radius = turning_radius + (trackWidth / 2.0f);

    // Calculate speed ratio (inner wheel should be slower)
    float speed_ratio = inner_radius / outer_radius;

    // Apply differential speeds
    if (steeringAngleRad > 0) // Turning left
    {
      // Left wheel is inner (slower), right wheel is outer (faster)
      rightMotor.setSpeed(speed);
      leftMotor.setSpeed(-speed * speed_ratio);
    }
    else // Turning right
    {
      // Right wheel is inner (slower), left wheel is outer (faster)
      rightMotor.setSpeed(speed * speed_ratio);
      leftMotor.setSpeed(-speed);
    }
  }
}

bool Car::isDriverHealthy()
{
  // Simple timeout check - if no motor updates in 1 second, consider unhealthy
  return (millis() - last_motor_update_) < 1000;
}

void Car::emergencyStop()
{
  // Immediate stop
  speed = 0.0f;
  steeringAngle = 0.0f;
  rightMotor.setSpeed(0.0f);
  leftMotor.setSpeed(0.0f);
  steeringMotor.setTargetAngle(0.0f);
  last_motor_update_ = millis(); // Reset health timer
}

void Car::updateControlLoops()
{
  // Update motor control loops directly
  rightMotor.updateControlLoop();
  leftMotor.updateControlLoop();
  steeringMotor.updatePosition();
}

void Car::begin()
{
  rightMotor.begin();
  delay(100); // Allow time for driver initialization
  leftMotor.begin();
  delay(100); // Allow time for driver initialization
  steeringMotor.begin();
  delay(100); // Allow time for driver initialization
}

void Car::setSteeringAngle(float angle)
{
  // Direct control - bypass queue for immediate response
  steeringAngle = angle;
  steeringMotor.setTargetAngle(angle);
  last_steering_update_ = millis();
}

void Car::setSpeed(float rpm, float wheelbase, float trackWidth)
{
  // Store geometry parameters
  this->wheelbase = wheelbase;
  this->trackWidth = trackWidth;
  speed = rpm;

  // Apply motor speeds with software differential
  applyMotorSpeeds();
  last_motor_update_ = millis();
}

float Car::getRightMotorRPM() const
{
  return rightMotor.getCurrentRPM();
}

float Car::getLeftMotorRPM() const
{
  return leftMotor.getCurrentRPM();
}
