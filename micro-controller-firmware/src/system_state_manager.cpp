#include "system_state_manager.h"
#include "connection_manager.h"
#include "recovery_manager.h"
#include "car.h"
#include <Arduino.h>
#include <algorithm>

// Default configuration values
static const uint32_t DEFAULT_CONNECTION_LOSS_TIMEOUT_MS = 5000;
static const uint32_t DEFAULT_DEGRADED_MODE_TIMEOUT_MS = 30000;
static const float DEFAULT_MAX_DEGRADED_SPEED = 100.0f; // RPM
static const float DEFAULT_MAX_DEGRADED_STEERING_ANGLE = 15.0f; // degrees
static const uint32_t DEFAULT_CACHE_VALIDITY_MS = 2000;

SystemStateManager::SystemStateManager(ConnectionManager* conn_mgr, RecoveryManager* recovery_mgr, Car* car_instance)
    : connection_manager_(conn_mgr)
    , recovery_manager_(recovery_mgr)
    , car_(car_instance)
{
    // Initialize system state
    current_state_.mode = SAFE_MODE;
    current_state_.available_capabilities = 0;
    current_state_.mode_start_time = millis();
    current_state_.last_full_operation_time = 0;
    current_state_.persistent_state_valid = false;
    current_state_.last_speed_command = 0.0f;
    current_state_.last_steering_command = 0.0f;
    current_state_.command_timestamp = 0;
    current_state_.uptime_ms = 0;
    current_state_.initialization_complete = false;
    
    // Initialize configuration with defaults
    config_.connection_loss_timeout_ms = DEFAULT_CONNECTION_LOSS_TIMEOUT_MS;
    config_.degraded_mode_timeout_ms = DEFAULT_DEGRADED_MODE_TIMEOUT_MS;
    config_.max_degraded_speed = DEFAULT_MAX_DEGRADED_SPEED;
    config_.max_degraded_steering_angle = DEFAULT_MAX_DEGRADED_STEERING_ANGLE;
    config_.allow_cached_operation = true;
    config_.cache_validity_ms = DEFAULT_CACHE_VALIDITY_MS;
    
    // Initialize cached command
    last_valid_command_ = {0.0f, 0.0f, 0, false};
}

void SystemStateManager::update() {
    uint32_t now = millis();
    current_state_.uptime_ms = now;
    
    // Update operational mode based on system conditions
    updateOperationalMode();
    
    // Update available capabilities
    updateCapabilities();
    
    // Apply current mode restrictions
    applyModeRestrictions();
    
    // Execute minimal safety checks
    executeMinimalSafetyChecks();
    
    // Handle automatic transitions
    OperationalMode target_mode = getCurrentMode();
    
    // Check for mode transitions based on system health
    if (connection_manager_->isHealthy() && !recovery_manager_->isSafeModeActive()) {
        if (current_state_.mode != FULL_OPERATION) {
            target_mode = FULL_OPERATION;
        }
    } else if (connection_manager_->getState() == ConnectionManager::CONNECTED) {
        if (current_state_.mode == SAFE_MODE || current_state_.mode == EMERGENCY_MODE) {
            target_mode = DEGRADED_OPERATION;
        }
    } else {
        if (current_state_.mode == FULL_OPERATION || current_state_.mode == DEGRADED_OPERATION) {
            target_mode = SAFE_MODE;
        }
    }
    
    // Apply transition if needed
    if (target_mode != current_state_.mode && shouldTransitionToMode(target_mode)) {
        transitionToMode(target_mode);
    }
}

void SystemStateManager::transitionToMode(OperationalMode new_mode) {
    if (new_mode == current_state_.mode) {
        return;
    }
    
    OperationalMode old_mode = current_state_.mode;
    
    // Cleanup current mode
    cleanupMode(old_mode);
    
    // Log transition
    logModeTransition(old_mode, new_mode);
    
    // Update state
    current_state_.mode = new_mode;
    current_state_.mode_start_time = millis();
    
    // Initialize new mode
    initializeMode(new_mode);
    
    // Update last full operation time
    if (new_mode == FULL_OPERATION) {
        current_state_.last_full_operation_time = millis();
    }
}

SystemStateManager::OperationalMode SystemStateManager::getCurrentMode() const {
    return current_state_.mode;
}

bool SystemStateManager::isCapabilityAvailable(SystemCapability capability) const {
    return (current_state_.available_capabilities & capability) != 0;
}

void SystemStateManager::enableCapability(SystemCapability capability) {
    current_state_.available_capabilities |= capability;
}

void SystemStateManager::disableCapability(SystemCapability capability) {
    current_state_.available_capabilities &= ~capability;
}

uint8_t SystemStateManager::getAvailableCapabilities() const {
    return current_state_.available_capabilities;
}

void SystemStateManager::persistCurrentState() {
    current_state_.persistent_state_valid = true;
    current_state_.command_timestamp = millis();
    
    // Additional persistence logic could be added here
    // For example, saving to EEPROM or flash memory
}

bool SystemStateManager::restorePersistedState() {
    if (!hasValidPersistedState()) {
        return false;
    }
    
    // Restore commands gradually for safety
    if (car_ != nullptr && canAcceptCommands()) {
        processSteeringCommand(current_state_.last_steering_command);
        // Speed restoration would be done gradually
    }
    
    return true;
}

void SystemStateManager::clearPersistedState() {
    current_state_.persistent_state_valid = false;
    current_state_.command_timestamp = 0;
    last_valid_command_.valid = false;
}

bool SystemStateManager::hasValidPersistedState() const {
    if (!current_state_.persistent_state_valid) {
        return false;
    }
    
    // Check if persisted state is recent enough
    uint32_t now = millis();
    return (now - current_state_.command_timestamp) < config_.cache_validity_ms;
}

void SystemStateManager::handleConnectionLoss() {
    // Persist current state before degrading
    persistCurrentState();
    
    // Transition to safe mode
    transitionToMode(SAFE_MODE);
    
    // Cache last valid command for potential fallback
    if (car_ != nullptr) {
        cacheValidCommand(current_state_.last_speed_command, current_state_.last_steering_command);
    }
}

void SystemStateManager::handlePartialFailure(SystemCapability failed_capability) {
    // Disable the failed capability
    disableCapability(failed_capability);
    
    // Determine if we need to degrade operation mode
    if (current_state_.mode == FULL_OPERATION) {
        // Check if we still have essential capabilities
        bool has_motor_control = isCapabilityAvailable(MOTOR_CONTROL);
        bool has_steering_control = isCapabilityAvailable(STEERING_CONTROL);
        
        if (!has_motor_control || !has_steering_control) {
            transitionToMode(SAFE_MODE);
        } else {
            transitionToMode(DEGRADED_OPERATION);
        }
    }
}

void SystemStateManager::handleSystemRecovery() {
    // Re-enable capabilities as they become available
    updateCapabilities();
    
    // Attempt to restore persisted state
    restorePersistedState();
    
    // Transition to appropriate mode based on available capabilities
    if (connection_manager_->isHealthy() && 
        isCapabilityAvailable(MOTOR_CONTROL) && 
        isCapabilityAvailable(STEERING_CONTROL)) {
        transitionToMode(FULL_OPERATION);
    } else if (connection_manager_->getState() == ConnectionManager::CONNECTED) {
        transitionToMode(DEGRADED_OPERATION);
    }
}

bool SystemStateManager::executeFallbackOperation() {
    if (!config_.allow_cached_operation || !isCommandCacheValid()) {
        return false;
    }
    
    // Execute cached commands with safety limits
    if (car_ != nullptr) {
        float safe_speed = std::min(last_valid_command_.speed, config_.max_degraded_speed);
        float safe_steering = std::max(-config_.max_degraded_steering_angle, 
                                      std::min(config_.max_degraded_steering_angle, 
                                              last_valid_command_.steering_angle));
        
        car_->setSpeed(safe_speed, 0.185f, 0.15f); // Use default values
        car_->setSteeringAngle(safe_steering);
        
        return true;
    }
    
    return false;
}

void SystemStateManager::maintainMinimalOperation() {
    if (car_ == nullptr) {
        return;
    }
    
    // Maintain basic motor control to prevent drift
    switch (current_state_.mode) {
        case SAFE_MODE:
            // Stop all motion
            car_->setSpeed(0.0f, 0.185f, 0.15f);
            car_->setSteeringAngle(0.0f);
            break;
            
        case DEGRADED_OPERATION:
            // Allow limited operation
            if (config_.allow_cached_operation && isCommandCacheValid()) {
                executeFallbackOperation();
            } else {
                car_->setSpeed(0.0f, 0.185f, 0.15f);
                car_->setSteeringAngle(0.0f);
            }
            break;
            
        case EMERGENCY_MODE:
            // Emergency stop
            car_->setSpeed(0.0f, 0.185f, 0.15f);
            car_->setSteeringAngle(0.0f);
            break;
            
        default:
            break;
    }
}

void SystemStateManager::executeEmergencyProtocol() {
    // Immediate transition to emergency mode
    transitionToMode(EMERGENCY_MODE);
    
    // Stop all operations
    if (car_ != nullptr) {
        car_->setSpeed(0.0f, 0.185f, 0.15f);
        car_->setSteeringAngle(0.0f);
    }
    
    // Clear all cached commands
    clearPersistedState();
    
    // Disable all non-essential capabilities
    current_state_.available_capabilities = 0;
}

void SystemStateManager::setDegradationConfig(const DegradationConfig& config) {
    config_ = config;
}

SystemStateManager::DegradationConfig SystemStateManager::getDegradationConfig() const {
    return config_;
}

SystemStateManager::SystemState SystemStateManager::getSystemState() const {
    return current_state_;
}

uint32_t SystemStateManager::getTimeInCurrentMode() const {
    return millis() - current_state_.mode_start_time;
}

uint32_t SystemStateManager::getTimeSinceFullOperation() const {
    if (current_state_.last_full_operation_time == 0) {
        return 0;
    }
    return millis() - current_state_.last_full_operation_time;
}

bool SystemStateManager::isSystemHealthy() const {
    return (current_state_.mode == FULL_OPERATION || current_state_.mode == DEGRADED_OPERATION) &&
           connection_manager_->isHealthy() &&
           !recovery_manager_->isEmergencyStopActive();
}

bool SystemStateManager::processSpeedCommand(float speed) {
    if (!canAcceptCommands()) {
        return false;
    }
    
    // Apply mode-specific limits
    float limited_speed = speed;
    switch (current_state_.mode) {
        case DEGRADED_OPERATION:
            limited_speed = std::min(std::abs(speed), config_.max_degraded_speed);
            if (speed < 0) limited_speed = -limited_speed;
            break;
            
        case SAFE_MODE:
        case EMERGENCY_MODE:
            limited_speed = 0.0f;
            break;
            
        default:
            break;
    }
    
    // Validate and execute command
    if (validateCommand(limited_speed, current_state_.last_steering_command)) {
        current_state_.last_speed_command = limited_speed;
        cacheValidCommand(limited_speed, current_state_.last_steering_command);
        
        if (car_ != nullptr) {
            car_->setSpeed(limited_speed, 0.185f, 0.15f);
        }
        
        return true;
    }
    
    return false;
}

bool SystemStateManager::processSteeringCommand(float angle) {
    if (!canAcceptCommands()) {
        return false;
    }
    
    // Apply mode-specific limits
    float limited_angle = angle;
    switch (current_state_.mode) {
        case DEGRADED_OPERATION:
            limited_angle = std::max(-config_.max_degraded_steering_angle,
                                   std::min(config_.max_degraded_steering_angle, angle));
            break;
            
        case SAFE_MODE:
        case EMERGENCY_MODE:
            limited_angle = 0.0f;
            break;
            
        default:
            break;
    }
    
    // Validate and execute command
    if (validateCommand(current_state_.last_speed_command, limited_angle)) {
        current_state_.last_steering_command = limited_angle;
        cacheValidCommand(current_state_.last_speed_command, limited_angle);
        
        if (car_ != nullptr) {
            car_->setSteeringAngle(limited_angle);
        }
        
        return true;
    }
    
    return false;
}

bool SystemStateManager::canAcceptCommands() const {
    return current_state_.mode != EMERGENCY_MODE &&
           isCapabilityAvailable(MOTOR_CONTROL) &&
           isCapabilityAvailable(STEERING_CONTROL);
}

void SystemStateManager::updateOperationalMode() {
    // This method determines the appropriate operational mode
    // based on current system conditions
}

void SystemStateManager::updateCapabilities() {
    // Reset capabilities
    current_state_.available_capabilities = 0;
    
    // Check connection-dependent capabilities
    if (connection_manager_->getState() == ConnectionManager::CONNECTED) {
        enableCapability(ROS_COMMUNICATION);
        
        if (connection_manager_->isHealthy()) {
            enableCapability(DEBUG_LOGGING);
            enableCapability(ODOMETRY_PUBLISHING);
        }
    }
    
    // Check hardware-dependent capabilities
    if (car_ != nullptr && current_state_.initialization_complete) {
        enableCapability(MOTOR_CONTROL);
        enableCapability(STEERING_CONTROL);
        enableCapability(SENSOR_READING);
    }
    
    // Disable capabilities in emergency mode
    if (current_state_.mode == EMERGENCY_MODE) {
        current_state_.available_capabilities = 0;
    }
}

void SystemStateManager::applyModeRestrictions() {
    switch (current_state_.mode) {
        case FULL_OPERATION:
            // No restrictions
            break;
            
        case DEGRADED_OPERATION:
            // Limited speed and steering
            maintainMinimalOperation();
            break;
            
        case SAFE_MODE:
            // Stop motion, maintain position
            maintainMinimalOperation();
            break;
            
        case EMERGENCY_MODE:
            // Emergency stop all operations
            maintainMinimalOperation();
            break;
    }
}

bool SystemStateManager::shouldTransitionToMode(OperationalMode mode) const {
    uint32_t time_in_mode = getTimeInCurrentMode();
    
    // Prevent rapid mode switching
    if (time_in_mode < 1000) { // Minimum 1 second in mode
        return false;
    }
    
    // Additional transition logic based on mode
    switch (mode) {
        case FULL_OPERATION:
            return connection_manager_->isHealthy() && 
                   !recovery_manager_->isSafeModeActive();
            
        case DEGRADED_OPERATION:
            return connection_manager_->getState() == ConnectionManager::CONNECTED;
            
        case SAFE_MODE:
            return true; // Always allow transition to safe mode
            
        case EMERGENCY_MODE:
            return true; // Always allow transition to emergency mode
            
        default:
            return false;
    }
}

void SystemStateManager::logModeTransition(OperationalMode old_mode, OperationalMode new_mode) {
    // This would integrate with the debug system
    // For now, we'll keep it simple since logging is done via ROS
}

void SystemStateManager::initializeMode(OperationalMode mode) {
    switch (mode) {
        case FULL_OPERATION:
            // Enable all capabilities
            updateCapabilities();
            break;
            
        case DEGRADED_OPERATION:
            // Enable limited capabilities
            updateCapabilities();
            break;
            
        case SAFE_MODE:
            // Minimal capabilities only
            disableCapability(ODOMETRY_PUBLISHING);
            break;
            
        case EMERGENCY_MODE:
            // No capabilities
            current_state_.available_capabilities = 0;
            break;
    }
}

void SystemStateManager::cleanupMode(OperationalMode mode) {
    // Perform any necessary cleanup when leaving a mode
    switch (mode) {
        case EMERGENCY_MODE:
            // Clear emergency state
            break;
            
        default:
            break;
    }
}

bool SystemStateManager::validateCommand(float speed, float steering_angle) const {
    // Basic validation - could be extended with more sophisticated checks
    return std::abs(speed) <= 1000.0f && std::abs(steering_angle) <= 90.0f;
}

void SystemStateManager::cacheValidCommand(float speed, float steering_angle) {
    last_valid_command_.speed = speed;
    last_valid_command_.steering_angle = steering_angle;
    last_valid_command_.timestamp = millis();
    last_valid_command_.valid = true;
}

bool SystemStateManager::isCommandCacheValid() const {
    if (!last_valid_command_.valid) {
        return false;
    }
    
    uint32_t now = millis();
    return (now - last_valid_command_.timestamp) < config_.cache_validity_ms;
}

void SystemStateManager::executeMinimalSafetyChecks() {
    // Perform continuous safety monitoring
    if (recovery_manager_->isEmergencyStopActive() && 
        current_state_.mode != EMERGENCY_MODE) {
        executeEmergencyProtocol();
    }
}