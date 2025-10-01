#include "recovery_manager.h"
#include "car.h"
#include <Arduino.h>
#include <algorithm>

// Default configuration values
static const uint32_t DEFAULT_RECOVERY_TIMEOUT_MS = 30000;
static const uint8_t DEFAULT_MAX_RECOVERY_ATTEMPTS = 3;
static const uint32_t DEFAULT_EMERGENCY_STOP_TIMEOUT_MS = 5000;

RecoveryManager::RecoveryManager(ConnectionManager* conn_mgr, Car* car_instance)
    : connection_manager_(conn_mgr)
    , car_(car_instance)
    , current_safe_mode_(NORMAL_OPERATION)
    , safe_mode_start_time_(0)
    , emergency_stop_active_(false)
    , emergency_stop_time_(0)
    , recovery_in_progress_(false)
    , recovery_start_time_(0)
    , recovery_timeout_ms_(DEFAULT_RECOVERY_TIMEOUT_MS)
    , max_recovery_attempts_(DEFAULT_MAX_RECOVERY_ATTEMPTS)
    , current_recovery_attempt_(0)
    , emergency_stop_timeout_ms_(DEFAULT_EMERGENCY_STOP_TIMEOUT_MS)
{
    // Initialize metrics
    metrics_ = {0, 0, 0, 0, 0, 0, 0.0f};
    
    // Initialize persisted state
    persisted_state_ = {0.0f, 0.0f, 0, false};
}

void RecoveryManager::update() {
    uint32_t now = millis();
    
    // Monitor system health and connection state
    monitorSystemHealth();
    
    // Update safe mode operations
    updateSafeModeOperations();
    
    // Handle emergency stop timeout
    if (emergency_stop_active_ && (now - emergency_stop_time_ > emergency_stop_timeout_ms_)) {
        // Emergency stop has been active long enough, try to recover
        if (connection_manager_->getState() == ConnectionManager::CONNECTED) {
            emergency_stop_active_ = false;
            exitSafeMode();
        }
    }
    
    // Handle recovery timeout
    if (recovery_in_progress_ && isRecoveryTimeout()) {
        logRecoveryEvent("recovery_timeout", false);
        updateMetrics(false);
        recovery_in_progress_ = false;
        
        // Escalate to higher safe mode level
        if (current_safe_mode_ < CRITICAL_ERROR) {
            enterSafeMode(static_cast<SafeModeLevel>(current_safe_mode_ + 1));
        }
    }
}

bool RecoveryManager::attemptRecovery() {
    if (recovery_in_progress_) {
        return false; // Recovery already in progress
    }
    
    recovery_in_progress_ = true;
    recovery_start_time_ = millis();
    current_recovery_attempt_ = 0;
    
    logRecoveryEvent("recovery_started", true);
    
    return executeGraduatedRecovery();
}

void RecoveryManager::handleConnectionDrop() {
    logRecoveryEvent("connection_drop_detected", true);
    
    // Save current system state before entering safe mode
    saveSystemState();
    
    // Enter appropriate safe mode
    enterSafeMode(CONNECTION_LOST);
    
    // Stop motors safely
    safeMotorStop();
    
    // Attempt automatic recovery
    attemptRecovery();
}

void RecoveryManager::handleCriticalError() {
    logRecoveryEvent("critical_error_detected", true);
    
    // Immediate emergency stop
    emergencyStop();
    
    // Enter critical error safe mode
    enterSafeMode(CRITICAL_ERROR);
    
    // Clear any ongoing recovery attempts
    recovery_in_progress_ = false;
}

void RecoveryManager::enterSafeMode(SafeModeLevel level) {
    if (level > current_safe_mode_) {
        current_safe_mode_ = level;
        safe_mode_start_time_ = millis();
        metrics_.safe_mode_activations++;
        
        // Apply safe mode restrictions
        applySafeModeRestrictions();
        
        logRecoveryEvent("safe_mode_entered", true);
    }
}

void RecoveryManager::exitSafeMode() {
    if (current_safe_mode_ != NORMAL_OPERATION) {
        current_safe_mode_ = NORMAL_OPERATION;
        safe_mode_start_time_ = 0;
        
        // Restore system state if available
        restoreSystemState();
        
        logRecoveryEvent("safe_mode_exited", true);
    }
}

RecoveryManager::SafeModeLevel RecoveryManager::getCurrentSafeModeLevel() const {
    return current_safe_mode_;
}

bool RecoveryManager::isSafeModeActive() const {
    return current_safe_mode_ != NORMAL_OPERATION;
}

void RecoveryManager::emergencyStop() {
    emergency_stop_active_ = true;
    emergency_stop_time_ = millis();
    metrics_.emergency_stops++;
    
    // Immediate motor stop
    safeMotorStop();
    
    logRecoveryEvent("emergency_stop_activated", true);
}

void RecoveryManager::safeMotorStop() {
    if (car_ != nullptr) {
        // Stop all motor operations safely
        car_->setSpeed(0.0f, 0.185f, 0.15f); // Use default wheelbase/track values
        car_->setSteeringAngle(0.0f);
        
        // Additional safety: ensure motors are actually stopped
        // This would be implemented in the Car class
    }
}

bool RecoveryManager::isEmergencyStopActive() const {
    return emergency_stop_active_;
}

bool RecoveryManager::executeGraduatedRecovery() {
    current_recovery_attempt_++;
    metrics_.total_recovery_attempts++;
    
    bool success = false;
    
    switch (current_recovery_attempt_) {
        case 1:
            // First attempt: Simple reconnection
            logRecoveryEvent("attempting_simple_reconnection", true);
            connection_manager_->forceReconnect();
            delay(1000); // Give time for reconnection
            success = (connection_manager_->getState() == ConnectionManager::CONNECTED);
            break;
            
        case 2:
            // Second attempt: Connection reset
            logRecoveryEvent("attempting_connection_reset", true);
            success = executeConnectionReset();
            break;
            
        case 3:
            // Third attempt: System restart
            logRecoveryEvent("attempting_system_restart", true);
            success = executeSystemRestart();
            break;
            
        default:
            // Max attempts reached
            logRecoveryEvent("max_recovery_attempts_reached", false);
            success = false;
            break;
    }
    
    if (success) {
        logRecoveryEvent("recovery_successful", true);
        updateMetrics(true);
        recovery_in_progress_ = false;
        
        // Exit safe mode if recovery was successful
        if (connection_manager_->isHealthy()) {
            exitSafeMode();
        }
    } else if (current_recovery_attempt_ >= max_recovery_attempts_) {
        logRecoveryEvent("recovery_failed", false);
        updateMetrics(false);
        recovery_in_progress_ = false;
        
        // Escalate safe mode level
        enterSafeMode(CRITICAL_ERROR);
    }
    
    return success;
}

bool RecoveryManager::executeConnectionReset() {
    // Reset connection manager state
    connection_manager_->resetErrorCounters();
    connection_manager_->forceReconnect();
    
    // Wait for connection to stabilize
    uint32_t start_time = millis();
    while (millis() - start_time < 5000) {
        connection_manager_->update();
        if (connection_manager_->getState() == ConnectionManager::CONNECTED) {
            return true;
        }
        delay(100);
    }
    
    return false;
}

bool RecoveryManager::executeSystemRestart() {
    // This would perform a more comprehensive system reset
    // For now, we'll do a thorough cleanup and reinitialize
    
    // Clear all error states
    connection_manager_->resetErrorCounters();
    
    // Reset recovery manager state
    current_safe_mode_ = NORMAL_OPERATION;
    emergency_stop_active_ = false;
    
    // Force reconnection
    connection_manager_->forceReconnect();
    
    // Wait longer for system to stabilize
    uint32_t start_time = millis();
    while (millis() - start_time < 10000) {
        connection_manager_->update();
        if (connection_manager_->isHealthy()) {
            return true;
        }
        delay(200);
    }
    
    return false;
}

void RecoveryManager::saveSystemState() {
    if (car_ != nullptr) {
        // Save current motor states
        persisted_state_.last_speed = 0.0f; // Would get from car if available
        persisted_state_.last_steering_angle = 0.0f; // Would get from car if available
        persisted_state_.timestamp = millis();
        persisted_state_.valid = true;
    }
}

bool RecoveryManager::restoreSystemState() {
    if (!persisted_state_.valid || car_ == nullptr) {
        return false;
    }
    
    // Only restore if the state is recent (within last 30 seconds)
    uint32_t now = millis();
    if (now - persisted_state_.timestamp > 30000) {
        clearPersistedState();
        return false;
    }
    
    // Restore motor states gradually for safety
    car_->setSteeringAngle(persisted_state_.last_steering_angle);
    // Speed restoration would be done gradually in actual implementation
    
    return true;
}

void RecoveryManager::clearPersistedState() {
    persisted_state_.valid = false;
    persisted_state_.timestamp = 0;
}

RecoveryManager::RecoveryMetrics RecoveryManager::getMetrics() const {
    RecoveryMetrics current_metrics = metrics_;
    
    // Calculate success rate
    if (current_metrics.total_recovery_attempts > 0) {
        current_metrics.recovery_success_rate = 
            (float)current_metrics.successful_recoveries / 
            (float)current_metrics.total_recovery_attempts;
    }
    
    return current_metrics;
}

void RecoveryManager::resetMetrics() {
    metrics_ = {0, 0, 0, 0, 0, 0, 0.0f};
}

uint32_t RecoveryManager::getTimeSinceLastRecovery() const {
    if (metrics_.last_recovery_time == 0) {
        return 0;
    }
    return millis() - metrics_.last_recovery_time;
}

void RecoveryManager::setRecoveryTimeout(uint32_t timeout_ms) {
    recovery_timeout_ms_ = timeout_ms;
}

void RecoveryManager::setMaxRecoveryAttempts(uint8_t max_attempts) {
    max_recovery_attempts_ = max_attempts;
}

void RecoveryManager::setEmergencyStopTimeout(uint32_t timeout_ms) {
    emergency_stop_timeout_ms_ = timeout_ms;
}

void RecoveryManager::updateSafeModeOperations() {
    if (!isSafeModeActive()) {
        return;
    }
    
    // Apply continuous safe mode restrictions
    applySafeModeRestrictions();
    
    // Monitor for conditions to exit safe mode
    if (current_safe_mode_ == CONNECTION_LOST) {
        if (connection_manager_->isHealthy()) {
            // Connection restored, can exit safe mode
            exitSafeMode();
        }
    }
}

void RecoveryManager::monitorSystemHealth() {
    // Determine appropriate safe mode level based on system state
    SafeModeLevel required_level = determineSafeModeLevel();
    
    if (required_level > current_safe_mode_) {
        enterSafeMode(required_level);
    }
}

bool RecoveryManager::isRecoveryTimeout() const {
    if (!recovery_in_progress_) {
        return false;
    }
    
    uint32_t now = millis();
    return (now - recovery_start_time_) > recovery_timeout_ms_;
}

void RecoveryManager::logRecoveryEvent(const char* event, bool success) {
    // This would integrate with the debug system
    // For now, we'll keep it simple since logging is done via ROS
}

void RecoveryManager::updateMetrics(bool recovery_success) {
    if (recovery_success) {
        metrics_.successful_recoveries++;
    } else {
        metrics_.failed_recoveries++;
    }
    
    metrics_.last_recovery_time = millis();
}

RecoveryManager::SafeModeLevel RecoveryManager::determineSafeModeLevel() const {
    ConnectionManager::State conn_state = connection_manager_->getState();
    
    switch (conn_state) {
        case ConnectionManager::CONNECTED:
            if (connection_manager_->isHealthy()) {
                return NORMAL_OPERATION;
            } else {
                return DEGRADED_CONNECTIVITY;
            }
            
        case ConnectionManager::DISCONNECTED:
        case ConnectionManager::WAITING:
        case ConnectionManager::AVAILABLE:
            return CONNECTION_LOST;
            
        case ConnectionManager::ERROR:
            return CRITICAL_ERROR;
            
        default:
            return CONNECTION_LOST;
    }
}

void RecoveryManager::applySafeModeRestrictions() {
    if (car_ == nullptr) {
        return;
    }
    
    switch (current_safe_mode_) {
        case NORMAL_OPERATION:
            // No restrictions
            break;
            
        case DEGRADED_CONNECTIVITY:
            // Reduce maximum speeds, increase safety margins
            // This would be implemented in the Car class
            break;
            
        case CONNECTION_LOST:
            // Stop all motion, maintain position
            safeMotorStop();
            break;
            
        case CRITICAL_ERROR:
            // Emergency stop, disable all operations
            emergencyStop();
            break;
    }
}

bool RecoveryManager::validateSystemState() const {
    // Validate that the system is in a safe state
    if (car_ == nullptr || connection_manager_ == nullptr) {
        return false;
    }
    
    // Check connection health
    if (current_safe_mode_ == NORMAL_OPERATION && 
        !connection_manager_->isHealthy()) {
        return false;
    }
    
    // Additional validation checks would go here
    return true;
}