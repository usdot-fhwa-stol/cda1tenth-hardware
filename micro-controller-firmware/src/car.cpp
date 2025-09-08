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

// Legacy Car class removed - using MultiCoreCar instead

// =============================================================================
// MOTOR CONTROL TASK IMPLEMENTATION
// =============================================================================

MotorControlTask::MotorControlTask() 
  : TaskInitializable("MotorControlTask"),
    rightMotor(nullptr), leftMotor(nullptr), steeringMotor(nullptr), carState(nullptr),
    taskHandle(nullptr), lastWakeTime(0), taskPeriodTicks(pdMS_TO_TICKS(1)) { // 1ms = 1kHz
  
  // Initialize performance metrics
  performance.init();
}

MotorControlTask::~MotorControlTask() {
  cleanup();
}

bool MotorControlTask::initialize(DriveMotor* rightMotor, DriveMotor* leftMotor, 
                                 SteeringMotor* steeringMotor, ThreadSafeCarState* carState,
                                 const SystemConfig_t& config) {
  // Store references
  this->rightMotor = rightMotor;
  this->leftMotor = leftMotor;
  this->steeringMotor = steeringMotor;
  this->carState = carState;
  this->config = config;
  
  // Use base class initialization
  return Initializable::initialize();
}

void MotorControlTask::cleanup() {
  Initializable::cleanup();
}

// Virtual method implementations from TaskInitializable
bool MotorControlTask::doInitialize() {
  // SPI mutex is already created in constructor via SpiMutex
  return true;
}

void MotorControlTask::doCleanup() {
  stopTask();
}

bool MotorControlTask::doHealthCheck() const {
  return isTaskHealthy();
}

bool MotorControlTask::createTask() {
  // Create the task
  BaseType_t result = xTaskCreatePinnedToCore(
    staticTaskFunction,
    "MotorControlTask",
    config.motor_task_stack_size,
    this,
    config.motor_task_priority,
    &taskHandle,
    config.motor_task_core_id
  );
  
  if (result != pdPASS) {
    return false;
  }
  
  lastWakeTime = xTaskGetTickCount();
  return true;
}

void MotorControlTask::destroyTask() {
  if (taskHandle != nullptr) {
    vTaskDelete(taskHandle);
    taskHandle = nullptr;
  }
}

void MotorControlTask::taskFunction() {
  // Initialize timing
  lastWakeTime = xTaskGetTickCount();
  
  while (isTaskRunning()) {
    uint32_t startTime = TimeUtils::getCurrentTimestampUs();
    
    // Execute control loop
    executeControlLoop();
    
    uint32_t executionTime = TimeUtils::getCurrentTimestampUs() - startTime;
    performance.update(executionTime);
    
    // Check for deadline miss
    if (isDeadlineMissed()) {
      performance.missed_deadlines++;
      taskErrorHandler.handleError("Task deadline missed");
    }
    
    // Wait for next period
    vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
  }
}

bool MotorControlTask::startTask() {
  return TaskInitializable::startTask();
}

void MotorControlTask::stopTask() {
  TaskInitializable::stopTask();
}

bool MotorControlTask::isRunning() {
  return isTaskRunning();
}


void MotorControlTask::executeControlLoop() {
  // Process incoming commands
  processMotorCommands();
  
  // Update motor control loops
  updateMotorControlLoops();
  
  // Report status (less frequently than control loop)
  static uint32_t lastStatusReport = 0;
  uint32_t currentTime = TimeUtils::getCurrentTimestampUs();
  if (currentTime - lastStatusReport >= 50000) { // 50ms
    reportMotorStatus();
    lastStatusReport = currentTime;
  }
}

void MotorControlTask::processMotorCommands() {
  if (carState == nullptr) return;
  
  MotorCommand_t cmd;
  if (carState->receiveCommandFromQueue(cmd, 0)) { // Non-blocking
    if (MotorValidation::validateMotorCommand(cmd)) {
      // Apply steering command
      MUTEX_LOCK(spiMutex);
      if (spiMutex.isTaken()) {
        steeringMotor->setTargetAngle(cmd.steering_angle);
      }
      
      // Apply speed command with differential drive calculation
      float speed_rpm = cmd.drive_speed * 60.0f / (M_PI * 0.06f); // Convert m/s to RPM
      const float steeringAngleRad = cmd.steering_angle * (M_PI / 180.0f);
      constexpr float minTurnAngle = 0.01f; // radians
      
      if (spiMutex.take()) {
        if (fabsf(steeringAngleRad) < minTurnAngle) {
          // Straight line motion
          rightMotor->setSpeed(speed_rpm);
          leftMotor->setSpeed(-speed_rpm);
        } else {
          // Differential drive calculation
          float R = cmd.wheelbase / tanf(steeringAngleRad);
          float R_L = R - (cmd.track_width / 2.0f);
          float R_R = R + (cmd.track_width / 2.0f);
          
          float v_L = speed_rpm * (R_L / R);
          float v_R = speed_rpm * (R_R / R);
          
          rightMotor->setSpeed(v_R);
          leftMotor->setSpeed(-v_L); // Left motor requires opposite rotation
        }
        spiMutex.give();
      }
    } else {
      taskErrorHandler.handleError("Invalid motor command received");
    }
  }
}

void MotorControlTask::updateMotorControlLoops() {
  if (spiMutex.take()) {
    rightMotor->updateControlLoop();
    leftMotor->updateControlLoop();
    steeringMotor->updatePosition();
    spiMutex.give();
  } else {
    performance.spi_errors++;
    taskErrorHandler.handleError("Mutex timeout");
  }
}

void MotorControlTask::reportMotorStatus() {
  if (carState == nullptr) return;
  
  MotorStatus_t status;
  motor_status_init(&status);
  
  if (spiMutex.take()) {
    status.right_motor_rpm = rightMotor->getCurrentRPM();
    status.left_motor_rpm = leftMotor->getCurrentRPM();
    status.steering_angle = steeringMotor->getSteeringAngle();
    status.steering_position = analogRead(STEERING_SENSOR_PIN);
    spiMutex.give();
  }
  
  status.timestamp = TimeUtils::getCurrentTimestampUs();
  status.motor_control_active = isHealthy();
  status.sequence_id = (status.sequence_id + 1) % 256;
  
  // Send status to queue
  if (!carState->sendStatusToQueue(status)) {
    performance.queue_overruns++;
    taskErrorHandler.handleError("Queue overflow");
  }
}

bool MotorControlTask::isDeadlineMissed() {
  TickType_t currentTime = xTaskGetTickCount();
  return (currentTime - lastWakeTime) > taskPeriodTicks;
}

MotorControlPerformanceMetrics MotorControlTask::getPerformanceMetrics() {
  return performance;
}

void MotorControlTask::resetPerformanceMetrics() {
  performance.reset();
}

bool MotorControlTask::isHealthy() const {
  return TaskInitializable::isHealthy();
}

uint32_t MotorControlTask::getErrorCount() {
  return taskErrorHandler.getErrorCount();
}

// =============================================================================
// MULTI-CORE COMPATIBLE CAR CLASS IMPLEMENTATION
// =============================================================================

MultiCoreCar::MultiCoreCar(int rightCS, int leftCS, int steerCS)
  : Initializable("MultiCoreCar"),
    rightMotor(rightCS), leftMotor(leftCS), steeringMotor(steerCS),
    carState(nullptr) {
  systemConfig = DEFAULT_SYSTEM_CONFIG;
}

MultiCoreCar::~MultiCoreCar() {
  cleanup();
}

bool MultiCoreCar::initialize() {
  // Get reference to inter-core communication
  carState = &InterCoreCommunication::getInstance().getCarState();
  
  // Use base class initialization
  return Initializable::initialize();
}

void MultiCoreCar::cleanup() {
  Initializable::cleanup();
}

// Virtual method implementations from Initializable
bool MultiCoreCar::doInitialize() {
  // Initialize motors
  if (!initializeMotors()) {
    return false;
  }
  
  // Initialize motor control task
  if (!motorControlTask.initialize(&rightMotor, &leftMotor, &steeringMotor, carState, systemConfig)) {
    return false;
  }
  
  // Start motor control task
  if (!motorControlTask.startTask()) {
    return false;
  }
  
  return true;
}

void MultiCoreCar::doCleanup() {
  // Stop motor control task
  motorControlTask.stopTask();
  motorControlTask.cleanup();
  
  // Clean up motors
  cleanupMotors();
}

bool MultiCoreCar::doHealthCheck() const {
  return motorControlTask.isHealthy();
}

// Motor control task is now handled by MotorControlTask class

bool MultiCoreCar::getCurrentStatus(MotorStatus_t& status) {
  if (!isInitialized() || carState == nullptr) {
    return false;
  }
  
  return carState->getStatus(status);
}

bool MultiCoreCar::isHealthy() {
  return Initializable::isHealthy();
}

void MultiCoreCar::setSystemConfig(const SystemConfig_t& config) {
  systemConfig = config;
}

MotorControlPerformanceMetrics MultiCoreCar::getPerformanceMetrics() {
  return motorControlTask.getPerformanceMetrics();
}

bool MultiCoreCar::initializeMotors() {
  if (!spiMutex.take(pdMS_TO_TICKS(1000))) {
    return false;
  }
  
  bool success = true;
  
  // Initialize SPI for motor drivers
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  delay(100);
  
  // Initialize motors
  rightMotor.begin();
  leftMotor.begin();
  steeringMotor.begin();
  
  // Set initial values
  rightMotor.setSpeed(0.0f);
  leftMotor.setSpeed(0.0f);
  steeringMotor.setTargetAngle(0.0f);
  
  spiMutex.give();
  return success;
}

void MultiCoreCar::cleanupMotors() {
  if (spiMutex.take(pdMS_TO_TICKS(1000))) {
    // Stop all motors
    rightMotor.setSpeed(0.0f);
    leftMotor.setSpeed(0.0f);
    steeringMotor.setTargetAngle(0.0f);
    
    spiMutex.give();
  }
}

// All motor control methods are now handled by MotorControlTask class
// All motor control methods are now handled by MotorControlTask class