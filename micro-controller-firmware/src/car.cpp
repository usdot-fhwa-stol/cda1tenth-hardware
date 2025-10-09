#include "car.h"
#include <limits.h>
#include <math.h>

SteeringMotor::SteeringMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {}

void SteeringMotor::begin()
{
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  driver.begin();
  driver.rms_current(300);
  driver.ihold(5);      // Hold torque when idle
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
  driver.RAMPMODE(2); // Velocity mode for continuous movement
  driver.X_ENC(0);
}

// TMC5160 recovery function from old working code
bool tmc5160_recover(TMC5160Stepper &drv, int ENN_PIN)
{
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

  return (gstat & (1 << 2)) == 0;
}

// Simplified recovery - just reset driver state
void simpleDriverReset(TMC5160Stepper &drv, int ENN_PIN)
{
  // Quick reset without complex state machine
  digitalWrite(ENN_PIN, HIGH);
  delayMicroseconds(100); // Very short delay
  drv.GSTAT(0b111);       // Clear faults
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
  { // Check for UV_CP fault
    if (!tmc5160_recover(driver, EN_PIN))
    {
      return; // Recovery failed, do not proceed
    }
  }

  int32_t current_enc = driver.X_ENC();
  uint32_t now = micros();
  uint32_t dt = now - last_time;
  int32_t delta_enc = current_enc - last_enc;

  // Avoid division by zero
  if (dt > 0)
  {
    float measured_ticks_per_sec = (float)delta_enc * 1e6f / dt;
    float measured_steps_per_sec = measured_ticks_per_sec * (MOTOR_STEPS * MICROSTEPS / ENCODER_TICKS_PER_REVOLUTION);
    measured_steps_per_sec = abs(measured_steps_per_sec);

    // Calculate current RPM
    current_rpm = (measured_steps_per_sec / (MOTOR_STEPS * MICROSTEPS)) * 60.0f;
  }

  // Only update motor if speed has changed
  if (step_rate_cmd != target_steps_per_sec)
  {
    // Direct speed control - no rate limiting
    step_rate_cmd = target_steps_per_sec;

    // Clamp to non-negative
    if (step_rate_cmd < 0.0f)
      step_rate_cmd = 0.0f;

    driver.VMAX(step_rate_cmd);
    driver.shaft(target_rpm < 0);
  }

  last_enc = current_enc;
  last_time = now;
}

bool DriveMotor::safeSPIOperation()
{
  // Quick fault check without blocking
  uint8_t gstat = driver.GSTAT();
  if (gstat & (1 << 2))
  { // UV_CP fault
    driver_ready = false;
    return false;
  }

  // Driver is ready
  driver_ready = true;

  // Set motor parameters with minimal SPI operations
  driver.VMAX(target_steps_per_sec);
  driver.shaft(target_rpm < 0);

  return true;
}

float DriveMotor::getCurrentRPM() const
{
  return current_rpm;
}

Car::Car(int rightCS, int leftCS, int steerCS)
    : rightMotor(rightCS), leftMotor(leftCS), steeringMotor(steerCS)
{
  // Initialize command queue
  clearQueue();
}

// Non-blocking command queue implementation
void Car::queueMotorCommand(float speed_rpm, float steering_angle)
{
  MotorCommand cmd;
  cmd.speed_rpm = speed_rpm;
  cmd.steering_angle = steering_angle;
  cmd.timestamp = millis();
  cmd.valid = true;

  addToQueue(cmd);
}

void Car::processCommandQueue()
{
  MotorCommand cmd;
  uint32_t now = millis();

  // Emergency stop if queue gets too full (indicates system overload)
  if (queue_count_ >= COMMAND_QUEUE_SIZE - 1)
  {
    emergencyStop();
    return;
  }

  // Limit queue processing to prevent system overload - reduced for dual-joystick
  int processed = 0;
  const int MAX_PROCESS_PER_CYCLE = 1; // Reduced from 2 to 1 for stability

  while (getFromQueue(cmd) && processed < MAX_PROCESS_PER_CYCLE)
  {
    processed++;

    // Process steering command with longer intervals for stability
    if (fabsf(cmd.steering_angle - steeringAngle) > 0.3f) // Increased threshold further
    {
      if (now - last_steering_update_ > 100) // Increased to 100ms for stability
      {
        steeringAngle = cmd.steering_angle;
        steeringMotor.setTargetAngle(cmd.steering_angle);
        last_steering_update_ = now;
      }
    }

    // Process speed command with longer intervals for stability
    if (fabsf(cmd.speed_rpm - speed) > 0.3f) // Increased threshold further
    {
      if (now - last_motor_update_ > 100) // Increased to 100ms for stability
      {
        speed = cmd.speed_rpm;
        rightMotor.setSpeed(speed);
        leftMotor.setSpeed(-speed);
        last_motor_update_ = now;
      }
    }
  }
}

bool Car::addToQueue(const MotorCommand &cmd)
{
  if (queue_count_ >= COMMAND_QUEUE_SIZE)
  {
    // Queue full, drop oldest command
    queue_head_ = (queue_head_ + 1) % COMMAND_QUEUE_SIZE;
    queue_count_--;
  }

  command_queue_[queue_tail_] = cmd;
  queue_tail_ = (queue_tail_ + 1) % COMMAND_QUEUE_SIZE;
  queue_count_++;
  return true;
}

bool Car::getFromQueue(MotorCommand &cmd)
{
  if (queue_count_ == 0)
  {
    return false;
  }

  cmd = command_queue_[queue_head_];
  queue_head_ = (queue_head_ + 1) % COMMAND_QUEUE_SIZE;
  queue_count_--;
  return true;
}

void Car::clearQueue()
{
  queue_head_ = 0;
  queue_tail_ = 0;
  queue_count_ = 0;
}

bool Car::isDriverHealthy()
{
  // Check if drivers are responding (non-blocking)
  uint32_t now = millis();

  // Check if we haven't updated motors in too long
  if (now - last_motor_update_ > 1000)
  { // 1 second timeout
    driver_healthy_ = false;
  }

  return driver_healthy_;
}

void Car::emergencyStop()
{
  // Immediate stop without queuing
  speed = 0.0f;
  steeringAngle = 0.0f;
  rightMotor.setSpeed(0.0f);
  leftMotor.setSpeed(0.0f);
  steeringMotor.setTargetAngle(0.0f);
  clearQueue();
  driver_healthy_ = true;
}

void Car::updateControlLoops()
{
  // Process queued motor commands
  processCommandQueue();

  // Update motor control loops directly (like old working code)
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
  // Queue the command for non-blocking operation
  queueMotorCommand(speed, angle);
}

void Car::setSpeed(float rpm, float wheelbase, float trackWidth)
{
  // Store geometry parameters
  this->wheelbase = wheelbase;
  this->trackWidth = trackWidth;
  speed = rpm;

  // steeringAngle is stored in degrees in this class; convert to radians for trig
  const float steeringAngleRad = steeringAngle * (M_PI / 180.0f);
  constexpr float minTurnAngle = 0.01f; // radians

  if (fabsf(steeringAngleRad) < minTurnAngle)
  {
    rightMotor.setSpeed(rpm);
    leftMotor.setSpeed(-rpm);
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
}

float Car::getRightMotorRPM() const
{
  return rightMotor.getCurrentRPM();
}

float Car::getLeftMotorRPM() const
{
  return leftMotor.getCurrentRPM();
}
