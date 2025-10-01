#include "car.h"
#include <limits.h>
#include <math.h>

SteeringMotor::SteeringMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {
  // Initialize health status
  health_status.current_fault = NO_FAULT;
  health_status.fault_count = 0;
  health_status.recovery_attempts = 0;
  health_status.last_fault_time = 0;
  health_status.is_healthy = true;
  health_status.health_score = 1.0f;
  health_status.position_error = 0.0f;
}

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

  // Non-blocking status check
  if (!checkStatusNonBlocking()) {
    return; // Skip position update if motor has faults
  }

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

void SteeringMotor::updatePosition(const SensorData& cached_data) {
  uint32_t now = micros();
  if (now - lastCorrectionMicros < STEERING_CORRECTION_INTERVAL) return;
  lastCorrectionMicros = now;

  // Non-blocking status check
  if (!checkStatusNonBlocking()) {
    return; // Skip position update if motor has faults
  }

  // Use cached steering angle if available and fresh
  float currentAngle;
  if (cached_data.valid && (millis() - cached_data.timestamp) < 100) {
    currentAngle = normalizeAngle(cached_data.steering_angle - angleOffset);
  } else {
    // Fallback to direct sensor reading
    currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  }
  
  float error = normalizeAngle(targetAngle - currentAngle);

  // Stall detection: if angle hasn't changed much, increment counter
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

bool SteeringMotor::checkStatusNonBlocking() {
  uint32_t now = millis();
  
  // Only check status periodically to avoid blocking
  if (now - last_status_check_time >= STATUS_CHECK_INTERVAL_MS) {
    last_status_check_time = now;
    
    // Comprehensive fault detection
    FaultType detected_fault = detectFaults();
    
    if (detected_fault != NO_FAULT) {
      health_status.current_fault = detected_fault;
      health_status.fault_count++;
      health_status.last_fault_time = now;
      health_status.is_healthy = false;
      
      // Attempt automatic recovery if within limits
      if (health_status.recovery_attempts < MAX_RECOVERY_ATTEMPTS &&
          (now - health_status.last_fault_time) > RECOVERY_COOLDOWN_MS) {
        last_status_check_result = recoverFromFault(detected_fault);
        health_status.recovery_attempts++;
      } else {
        last_status_check_result = false;
      }
    } else {
      health_status.current_fault = NO_FAULT;
      health_status.is_healthy = true;
      last_status_check_result = true;
    }
    
    updateHealthScore();
  }
  
  return last_status_check_result;
}

SteeringMotor::FaultType SteeringMotor::detectFaults() {
  // Check GSTAT register for various faults
  uint8_t gstat = driver.GSTAT();
  
  if (gstat & (1 << 2)) return UV_CP_FAULT;        // Charge pump undervoltage
  if (gstat & (1 << 1)) return OVERTEMP_FAULT;     // Overtemperature warning
  if (gstat & (1 << 0)) return COMMUNICATION_FAULT; // Reset flag
  
  // Check DRV_STATUS for additional faults
  uint32_t drv_status = driver.DRV_STATUS();
  
  if (drv_status & (1 << 31)) return SHORT_CIRCUIT_FAULT; // Short to ground A
  if (drv_status & (1 << 30)) return SHORT_CIRCUIT_FAULT; // Short to ground B
  if (drv_status & (1 << 27)) return OPEN_LOAD_FAULT;     // Open load A
  if (drv_status & (1 << 26)) return OPEN_LOAD_FAULT;     // Open load B
  
  // Check for stall condition
  if (stallCounter > STALL_DETECTION_COUNT) return STALL_FAULT;
  
  // Check for position error
  if (!validatePosition()) return POSITION_ERROR_FAULT;
  
  return NO_FAULT;
}

bool SteeringMotor::recoverFromFault(FaultType fault) {
  switch (fault) {
    case UV_CP_FAULT:
      return tmc5160_recover(driver, EN_PIN);
      
    case OVERTEMP_FAULT:
      // Reduce current and wait for cooldown
      driver.irun(25); // Reduce current
      delay(1000);
      driver.irun(50); // Restore current
      return true;
      
    case SHORT_CIRCUIT_FAULT:
    case OPEN_LOAD_FAULT:
      // Clear faults and reinitialize
      driver.GSTAT(0b111);
      delay(100);
      begin(); // Reinitialize motor
      return true;
      
    case STALL_FAULT:
      {
        // Reset stall counter and resync position
        stallCounter = 0;
        float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
        float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
        int32_t actualSteps = (int32_t)((currentAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);
        driver.XACTUAL(actualSteps);
        driver.XTARGET(actualSteps);
        return true;
      }
      
    case COMMUNICATION_FAULT:
      // Clear reset flag and reinitialize
      driver.GSTAT(1);
      delay(50);
      begin();
      return true;
      
    case POSITION_ERROR_FAULT:
      {
        // Force position resync
        float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
        float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
        int32_t actualSteps = (int32_t)((currentAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);
        driver.XACTUAL(actualSteps);
        return true;
      }
      
    default:
      return false;
  }
}

SteeringMotor::MotorHealth SteeringMotor::getHealthStatus() const {
  return health_status;
}

void SteeringMotor::updateHealthScore() {
  uint32_t now = millis();
  
  // Base health score calculation
  float base_score = 1.0f;
  
  // Reduce score based on fault frequency
  if (health_status.fault_count > 0) {
    base_score -= (health_status.fault_count * 0.1f);
  }
  
  // Reduce score based on recent faults
  if (health_status.last_fault_time > 0) {
    uint32_t time_since_fault = now - health_status.last_fault_time;
    if (time_since_fault < 60000) { // Less than 1 minute
      base_score -= 0.3f;
    } else if (time_since_fault < 300000) { // Less than 5 minutes
      base_score -= 0.1f;
    }
  }
  
  // Reduce score based on recovery attempts
  base_score -= (health_status.recovery_attempts * 0.15f);
  
  // Reduce score based on position error
  base_score -= (health_status.position_error * 0.01f);
  
  // Clamp to valid range
  health_status.health_score = max(0.0f, min(1.0f, base_score));
}

void SteeringMotor::resetFaultCounters() {
  health_status.fault_count = 0;
  health_status.recovery_attempts = 0;
  health_status.last_fault_time = 0;
  health_status.current_fault = NO_FAULT;
  health_status.is_healthy = true;
  health_status.health_score = 1.0f;
  health_status.position_error = 0.0f;
}

bool SteeringMotor::validatePosition() {
  float currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  float error = normalizeAngle(targetAngle - currentAngle);
  health_status.position_error = fabsf(error);
  
  // Consider position invalid if error exceeds threshold
  return health_status.position_error < (STEERING_MAX_ALLOWED_ERROR * 2.0f);
}

void SteeringMotor::applySpeed() {
  // Empty: internal logic controls speed automatically
}


DriveMotor::DriveMotor(int cs) : driver(cs, R_SENSE), cs_pin(cs) {
  // Initialize health status
  health_status.current_fault = NO_FAULT;
  health_status.fault_count = 0;
  health_status.recovery_attempts = 0;
  health_status.last_fault_time = 0;
  health_status.is_healthy = true;
  health_status.health_score = 1.0f;
}

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
  // Non-blocking status check
  if (!checkStatusNonBlocking()) {
    return; // Skip control loop if motor has faults
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

void DriveMotor::updateControlLoop(const SensorData& cached_data) {
  // Non-blocking status check
  if (!checkStatusNonBlocking()) {
    return; // Skip control loop if motor has faults
  }
  
  // Use cached RPM data if available and fresh
  if (cached_data.valid && (millis() - cached_data.timestamp) < 100) {
    // Use cached current RPM for this motor (determine which motor this is based on cs_pin)
    float cached_rpm = (cs_pin == CS_RIGHT) ? cached_data.right_rpm : cached_data.left_rpm;
    
    // Convert cached RPM to steps per second for control calculations
    float measured_steps_per_sec = (abs(cached_rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
    current_rpm = abs(cached_rpm);
    
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

    uint32_t now = millis();
    uint32_t dt = now - last_time;
    
    // Limit rate of change
    float max_step_change = MAX_STEP_ACCEL * (dt / 1000.0f);  // steps/sec

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
    
    last_time = now;
  } else {
    // Fallback to original encoder-based control loop
    updateControlLoop();
  }
}

bool DriveMotor::checkStatusNonBlocking() {
  uint32_t now = millis();
  
  // Only check status periodically to avoid blocking
  if (now - last_status_check_time >= STATUS_CHECK_INTERVAL_MS) {
    last_status_check_time = now;
    
    // Comprehensive fault detection
    FaultType detected_fault = detectFaults();
    
    if (detected_fault != NO_FAULT) {
      health_status.current_fault = detected_fault;
      health_status.fault_count++;
      health_status.last_fault_time = now;
      health_status.is_healthy = false;
      
      // Attempt automatic recovery if within limits
      if (health_status.recovery_attempts < MAX_RECOVERY_ATTEMPTS &&
          (now - health_status.last_fault_time) > RECOVERY_COOLDOWN_MS) {
        last_status_check_result = recoverFromFault(detected_fault);
        health_status.recovery_attempts++;
      } else {
        last_status_check_result = false;
      }
    } else {
      health_status.current_fault = NO_FAULT;
      health_status.is_healthy = true;
      last_status_check_result = true;
    }
    
    updateHealthScore();
  }
  
  return last_status_check_result;
}

DriveMotor::FaultType DriveMotor::detectFaults() {
  // Check GSTAT register for various faults
  uint8_t gstat = driver.GSTAT();
  
  if (gstat & (1 << 2)) return UV_CP_FAULT;        // Charge pump undervoltage
  if (gstat & (1 << 1)) return OVERTEMP_FAULT;     // Overtemperature warning
  if (gstat & (1 << 0)) return COMMUNICATION_FAULT; // Reset flag
  
  // Check DRV_STATUS for additional faults
  uint32_t drv_status = driver.DRV_STATUS();
  
  if (drv_status & (1 << 31)) return SHORT_CIRCUIT_FAULT; // Short to ground A
  if (drv_status & (1 << 30)) return SHORT_CIRCUIT_FAULT; // Short to ground B
  if (drv_status & (1 << 27)) return OPEN_LOAD_FAULT;     // Open load A
  if (drv_status & (1 << 26)) return OPEN_LOAD_FAULT;     // Open load B
  
  // Check for stall condition
  if (stall_counter > DRIVE_MAX_STALL_COUNT) return STALL_FAULT;
  
  return NO_FAULT;
}

bool DriveMotor::recoverFromFault(FaultType fault) {
  switch (fault) {
    case UV_CP_FAULT:
      return tmc5160_recover(driver, EN_PIN);
      
    case OVERTEMP_FAULT:
      // Reduce current and wait for cooldown
      driver.irun(15); // Reduce to half current
      delay(1000);
      driver.irun(31); // Restore full current
      return true;
      
    case SHORT_CIRCUIT_FAULT:
    case OPEN_LOAD_FAULT:
      // Clear faults and reinitialize
      driver.GSTAT(0b111);
      delay(100);
      begin(); // Reinitialize motor
      return true;
      
    case STALL_FAULT:
      // Reset stall counter and reduce speed temporarily
      stall_counter = 0;
      step_rate_cmd = step_rate_cmd * 0.8f; // Reduce speed by 20%
      return true;
      
    case COMMUNICATION_FAULT:
      // Clear reset flag and reinitialize
      driver.GSTAT(1);
      delay(50);
      begin();
      return true;
      
    default:
      return false;
  }
}

DriveMotor::MotorHealth DriveMotor::getHealthStatus() const {
  return health_status;
}

void DriveMotor::updateHealthScore() {
  uint32_t now = millis();
  
  // Base health score calculation
  float base_score = 1.0f;
  
  // Reduce score based on fault frequency
  if (health_status.fault_count > 0) {
    base_score -= (health_status.fault_count * 0.1f);
  }
  
  // Reduce score based on recent faults
  if (health_status.last_fault_time > 0) {
    uint32_t time_since_fault = now - health_status.last_fault_time;
    if (time_since_fault < 60000) { // Less than 1 minute
      base_score -= 0.3f;
    } else if (time_since_fault < 300000) { // Less than 5 minutes
      base_score -= 0.1f;
    }
  }
  
  // Reduce score based on recovery attempts
  base_score -= (health_status.recovery_attempts * 0.15f);
  
  // Clamp to valid range
  health_status.health_score = max(0.0f, min(1.0f, base_score));
}

void DriveMotor::resetFaultCounters() {
  health_status.fault_count = 0;
  health_status.recovery_attempts = 0;
  health_status.last_fault_time = 0;
  health_status.current_fault = NO_FAULT;
  health_status.is_healthy = true;
  health_status.health_score = 1.0f;
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
  unlock();
}

void Car::updateControlLoops(const SensorData& cached_data) {
  lock();
  rightMotor.updateControlLoop(cached_data, &spi_manager);
  leftMotor.updateControlLoop(cached_data, &spi_manager);
  steeringMotor.updatePosition(cached_data, &spi_manager);
  
  // Process any pending SPI operations
  spi_manager.processPendingOperations();
  unlock();
}

void Car::processSPIOperations() {
  lock();
  spi_manager.flushQueue();
  unlock();
}

SPIManager::SPIPerformanceMetrics Car::getSPIMetrics() const {
  return spi_manager.getMetrics();
}

bool Car::isSPIHealthy() const {
  return spi_manager.isHealthy();
}

void Car::enableAdaptiveSPIBatching(bool enable) {
  spi_manager.enableAdaptiveBatching(enable);
}

uint32_t Car::getSPIErrorCount() const {
  return spi_manager.getConsecutiveErrors();
}

void Car::clearSPIErrors() {
  spi_manager.clearErrorHistory();
}

void Car::begin() {
  spi_manager.initialize();
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

// SPI-optimized motor control methods
void DriveMotor::updateControlLoop(const SensorData& cached_data, SPIManager* spi_mgr) {
  // Non-blocking status check using SPI manager
  if (!checkStatusNonBlocking(spi_mgr)) {
    return; // Skip control loop if motor has faults
  }
  
  // Use cached RPM data if available and fresh
  if (cached_data.valid && (millis() - cached_data.timestamp) < 100) {
    // Use cached current RPM for this motor (determine which motor this is based on cs_pin)
    float cached_rpm = (cs_pin == CS_RIGHT) ? cached_data.right_rpm : cached_data.left_rpm;
    
    // Convert cached RPM to steps per second for control calculations
    float measured_steps_per_sec = (abs(cached_rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
    current_rpm = abs(cached_rpm);
    
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

    uint32_t now = millis();
    uint32_t dt = now - last_time;
    
    // Limit rate of change
    float max_step_change = MAX_STEP_ACCEL * (dt / 1000.0f);  // steps/sec

    if (target_steps_per_sec > step_rate_cmd + max_step_change) {
      step_rate_cmd += max_step_change;
    } else if (target_steps_per_sec < step_rate_cmd - max_step_change) {
      step_rate_cmd -= max_step_change;
    } else {
      step_rate_cmd = target_steps_per_sec;
    }

    // Clamp to non-negative
    if (step_rate_cmd < 0.0f) step_rate_cmd = 0.0f;

    // Queue SPI operations for batched execution
    if (spi_mgr) {
      // Use priority queue for emergency stops (speed = 0)
      bool is_emergency_stop = (step_rate_cmd == 0 && target_rpm == 0);
      
      SPIManager::SPIOperation vmax_op(cs_pin, SPIManager::WRITE_OPERATION, 0x24, step_rate_cmd);
      spi_mgr->queueOperation(vmax_op, is_emergency_stop);
      
      SPIManager::SPIOperation shaft_op(cs_pin, SPIManager::WRITE_OPERATION, 0x6C, target_rpm < 0 ? 1 : 0);
      spi_mgr->queueOperation(shaft_op, is_emergency_stop);
    } else {
      // Fallback to direct SPI
      driver.VMAX(step_rate_cmd);
      driver.shaft(target_rpm < 0);
    }
    
    last_time = now;
  } else {
    // Fallback to original encoder-based control loop
    updateControlLoop(cached_data);
  }
}

bool DriveMotor::checkStatusNonBlocking(SPIManager* spi_mgr) {
  uint32_t now = millis();
  
  // Only check status periodically to avoid blocking
  if (now - last_status_check_time >= STATUS_CHECK_INTERVAL_MS) {
    last_status_check_time = now;
    
    // Comprehensive fault detection using SPI manager
    FaultType detected_fault = detectFaults(spi_mgr);
    
    if (detected_fault != NO_FAULT) {
      health_status.current_fault = detected_fault;
      health_status.fault_count++;
      health_status.last_fault_time = now;
      health_status.is_healthy = false;
      
      // Attempt automatic recovery if within limits
      if (health_status.recovery_attempts < MAX_RECOVERY_ATTEMPTS &&
          (now - health_status.last_fault_time) > RECOVERY_COOLDOWN_MS) {
        last_status_check_result = recoverFromFault(detected_fault);
        health_status.recovery_attempts++;
      } else {
        last_status_check_result = false;
      }
    } else {
      health_status.current_fault = NO_FAULT;
      health_status.is_healthy = true;
      last_status_check_result = true;
    }
    
    updateHealthScore();
  }
  
  return last_status_check_result;
}

DriveMotor::FaultType DriveMotor::detectFaults(SPIManager* spi_mgr) {
  uint32_t gstat_result = 0;
  uint32_t drv_status_result = 0;
  
  if (spi_mgr) {
    // Queue non-blocking SPI operations
    SPIManager::SPIOperation gstat_op(cs_pin, SPIManager::READ_OPERATION, 0x01, 0, &gstat_result);
    SPIManager::SPIOperation drv_op(cs_pin, SPIManager::READ_OPERATION, 0x6F, 0, &drv_status_result);
    
    spi_mgr->queueOperation(gstat_op);
    spi_mgr->queueOperation(drv_op);
    spi_mgr->flushQueue();
    
    // Check if operations completed successfully
    if (!gstat_op.success || !drv_op.success) {
      return COMMUNICATION_FAULT;
    }
  } else {
    // Fallback to direct SPI
    gstat_result = driver.GSTAT();
    drv_status_result = driver.DRV_STATUS();
  }
  
  // Analyze results
  uint8_t gstat = gstat_result & 0xFF;
  
  if (gstat & (1 << 2)) return UV_CP_FAULT;        // Charge pump undervoltage
  if (gstat & (1 << 1)) return OVERTEMP_FAULT;     // Overtemperature warning
  if (gstat & (1 << 0)) return COMMUNICATION_FAULT; // Reset flag
  
  if (drv_status_result & (1 << 31)) return SHORT_CIRCUIT_FAULT; // Short to ground A
  if (drv_status_result & (1 << 30)) return SHORT_CIRCUIT_FAULT; // Short to ground B
  if (drv_status_result & (1 << 27)) return OPEN_LOAD_FAULT;     // Open load A
  if (drv_status_result & (1 << 26)) return OPEN_LOAD_FAULT;     // Open load B
  
  // Check for stall condition
  if (stall_counter > DRIVE_MAX_STALL_COUNT) return STALL_FAULT;
  
  return NO_FAULT;
}

void SteeringMotor::updatePosition(const SensorData& cached_data, SPIManager* spi_mgr) {
  uint32_t now = micros();
  if (now - lastCorrectionMicros < STEERING_CORRECTION_INTERVAL) return;
  lastCorrectionMicros = now;

  // Non-blocking status check using SPI manager
  if (!checkStatusNonBlocking(spi_mgr)) {
    return; // Skip position update if motor has faults
  }

  // Use cached steering angle if available and fresh
  float currentAngle;
  if (cached_data.valid && (millis() - cached_data.timestamp) < 100) {
    currentAngle = normalizeAngle(cached_data.steering_angle - angleOffset);
  } else {
    // Fallback to direct sensor reading
    currentAngle = normalizeAngle(getSteeringAngle() - angleOffset);
  }
  
  float error = normalizeAngle(targetAngle - currentAngle);

  // Stall detection: if angle hasn't changed much, increment counter
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
    if (spi_mgr) {
      SPIManager::SPIOperation xactual_op(cs_pin, SPIManager::WRITE_OPERATION, 0x21, actualSteps);
      SPIManager::SPIOperation xtarget_op(cs_pin, SPIManager::WRITE_OPERATION, 0x2D, actualSteps);
      spi_mgr->queueOperation(xactual_op);
      spi_mgr->queueOperation(xtarget_op);
    } else {
      driver.XACTUAL(actualSteps);
      driver.XTARGET(actualSteps);
    }
    stallCounter = 0;
    return;
  }

  // Normal drift correction if error exceeds threshold
  if (fabsf(error) > STEERING_MAX_ALLOWED_ERROR) {
    if (spi_mgr) {
      // Position corrections are important for safety, use priority queue
      bool is_critical_correction = fabsf(error) > (STEERING_MAX_ALLOWED_ERROR * 2.0f);
      
      SPIManager::SPIOperation xactual_op(cs_pin, SPIManager::WRITE_OPERATION, 0x21, actualSteps);
      spi_mgr->queueOperation(xactual_op, is_critical_correction);
    } else {
      driver.XACTUAL(actualSteps);
    }
  }
}

bool SteeringMotor::checkStatusNonBlocking(SPIManager* spi_mgr) {
  uint32_t now = millis();
  
  // Only check status periodically to avoid blocking
  if (now - last_status_check_time >= STATUS_CHECK_INTERVAL_MS) {
    last_status_check_time = now;
    
    // Comprehensive fault detection using SPI manager
    FaultType detected_fault = detectFaults(spi_mgr);
    
    if (detected_fault != NO_FAULT) {
      health_status.current_fault = detected_fault;
      health_status.fault_count++;
      health_status.last_fault_time = now;
      health_status.is_healthy = false;
      
      // Attempt automatic recovery if within limits
      if (health_status.recovery_attempts < MAX_RECOVERY_ATTEMPTS &&
          (now - health_status.last_fault_time) > RECOVERY_COOLDOWN_MS) {
        last_status_check_result = recoverFromFault(detected_fault);
        health_status.recovery_attempts++;
      } else {
        last_status_check_result = false;
      }
    } else {
      health_status.current_fault = NO_FAULT;
      health_status.is_healthy = true;
      last_status_check_result = true;
    }
    
    updateHealthScore();
  }
  
  return last_status_check_result;
}

SteeringMotor::FaultType SteeringMotor::detectFaults(SPIManager* spi_mgr) {
  uint32_t gstat_result = 0;
  uint32_t drv_status_result = 0;
  
  if (spi_mgr) {
    // Queue non-blocking SPI operations
    SPIManager::SPIOperation gstat_op(cs_pin, SPIManager::READ_OPERATION, 0x01, 0, &gstat_result);
    SPIManager::SPIOperation drv_op(cs_pin, SPIManager::READ_OPERATION, 0x6F, 0, &drv_status_result);
    
    spi_mgr->queueOperation(gstat_op);
    spi_mgr->queueOperation(drv_op);
    spi_mgr->flushQueue();
    
    // Check if operations completed successfully
    if (!gstat_op.success || !drv_op.success) {
      return COMMUNICATION_FAULT;
    }
  } else {
    // Fallback to direct SPI
    gstat_result = driver.GSTAT();
    drv_status_result = driver.DRV_STATUS();
  }
  
  // Analyze results
  uint8_t gstat = gstat_result & 0xFF;
  
  if (gstat & (1 << 2)) return UV_CP_FAULT;        // Charge pump undervoltage
  if (gstat & (1 << 1)) return OVERTEMP_FAULT;     // Overtemperature warning
  if (gstat & (1 << 0)) return COMMUNICATION_FAULT; // Reset flag
  
  if (drv_status_result & (1 << 31)) return SHORT_CIRCUIT_FAULT; // Short to ground A
  if (drv_status_result & (1 << 30)) return SHORT_CIRCUIT_FAULT; // Short to ground B
  if (drv_status_result & (1 << 27)) return OPEN_LOAD_FAULT;     // Open load A
  if (drv_status_result & (1 << 26)) return OPEN_LOAD_FAULT;     // Open load B
  
  // Check for stall condition
  if (stallCounter > STALL_DETECTION_COUNT) return STALL_FAULT;
  
  // Check for position error
  if (!validatePosition()) return POSITION_ERROR_FAULT;
  
  return NO_FAULT;
}