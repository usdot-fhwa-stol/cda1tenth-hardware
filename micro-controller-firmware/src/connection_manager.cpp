#include "connection_manager.h"
#include <Arduino.h>
#include <algorithm>

// Default configuration values
static const uint8_t DEFAULT_MAX_RETRIES = 5;
static const uint32_t DEFAULT_BASE_BACKOFF_MS = 1000;
static const uint32_t DEFAULT_MAX_BACKOFF_MS = 30000;
static const uint32_t DEFAULT_PING_INTERVAL_MS = 2000;
static const uint32_t PING_FAILURE_THRESHOLD = 3;
static const uint32_t CONNECTION_QUALITY_WINDOW = 10;

ConnectionManager::ConnectionManager() 
    : current_state_(WAITING)
    , previous_state_(WAITING)
    , state_change_time_(0)
    , connection_start_time_(0)
    , last_ping_time_(0)
    , last_successful_ping_(0)
    , ping_interval_ms_(DEFAULT_PING_INTERVAL_MS)
    , consecutive_ping_failures_(0)
    , total_ping_attempts_(0)
    , successful_pings_(0)
    , connection_drop_count_(0)
    , recovery_attempts_(0)
    , consecutive_failures_(0)
    , current_strategy_(RETRY)
    , max_retries_(DEFAULT_MAX_RETRIES)
    , base_backoff_ms_(DEFAULT_BASE_BACKOFF_MS)
    , max_backoff_ms_(DEFAULT_MAX_BACKOFF_MS)
    , current_backoff_ms_(DEFAULT_BASE_BACKOFF_MS)
    , last_retry_time_(0)
    , safe_mode_active_(false)
    , safe_mode_start_time_(0)
{
    state_change_time_ = millis();
}

void ConnectionManager::update() {
    uint32_t now = millis();
    
    // Update connection health monitoring
    updateConnectionHealth();
    
    switch (current_state_) {
        case WAITING:
            // Check if agent is available
            if (now - last_retry_time_ >= current_backoff_ms_) {
                if (pingAgent()) {
                    transitionToState(AVAILABLE);
                    current_backoff_ms_ = base_backoff_ms_; // Reset backoff on success
                } else {
                    handleConnectionFailure();
                }
                last_retry_time_ = now;
            }
            break;
            
        case AVAILABLE:
            // State managed externally - transition to CONNECTED when entities created
            break;
            
        case CONNECTED:
            // Regular health monitoring
            if (now - last_ping_time_ >= ping_interval_ms_) {
                if (!pingAgent()) {
                    consecutive_ping_failures_++;
                    if (consecutive_ping_failures_ >= PING_FAILURE_THRESHOLD) {
                        transitionToState(DISCONNECTED);
                        connection_drop_count_++;
                    }
                } else {
                    consecutive_ping_failures_ = 0;
                    last_successful_ping_ = now;
                }
                last_ping_time_ = now;
            }
            break;
            
        case DISCONNECTED:
            // Attempt recovery based on strategy
            if (shouldRetry()) {
                recovery_attempts_++;
                transitionToState(WAITING);
            } else if (now - state_change_time_ > max_backoff_ms_) {
                // Force transition to ERROR state if stuck too long
                transitionToState(ERROR);
            }
            break;
            
        case ERROR:
            // In error state, only manual recovery or timeout can help
            if (now - state_change_time_ > max_backoff_ms_ * 2) {
                // Reset to waiting after extended timeout
                resetErrorCounters();
                transitionToState(WAITING);
            }
            break;
    }
}

ConnectionManager::State ConnectionManager::getState() const {
    return current_state_;
}

bool ConnectionManager::isHealthy() const {
    if (current_state_ != CONNECTED) {
        return false;
    }
    
    uint32_t now = millis();
    uint32_t time_since_last_ping = now - last_successful_ping_;
    
    // Consider healthy if we've had a successful ping recently
    return time_since_last_ping < (ping_interval_ms_ * 2);
}

void ConnectionManager::forceReconnect() {
    transitionToState(WAITING);
    current_backoff_ms_ = base_backoff_ms_;
    last_retry_time_ = 0; // Force immediate retry
}

bool ConnectionManager::pingAgent(uint32_t timeout_ms) {
    total_ping_attempts_++;
    
    rmw_ret_t ret = rmw_uros_ping_agent(timeout_ms, 1);
    bool success = (ret == RMW_RET_OK);
    
    if (success) {
        successful_pings_++;
    }
    
    return success;
}

uint32_t ConnectionManager::getConnectionUptime() const {
    if (current_state_ != CONNECTED || connection_start_time_ == 0) {
        return 0;
    }
    return millis() - connection_start_time_;
}

uint32_t ConnectionManager::getLastPingTime() const {
    return last_ping_time_;
}

float ConnectionManager::getConnectionQuality() const {
    if (total_ping_attempts_ == 0) {
        return 0.0f;
    }
    
    // Calculate success rate over recent attempts
    uint32_t recent_attempts = std::min(total_ping_attempts_, (uint32_t)CONNECTION_QUALITY_WINDOW);
    uint32_t recent_successes = std::min(successful_pings_, recent_attempts);
    
    return (float)recent_successes / (float)recent_attempts;
}

uint32_t ConnectionManager::getConnectionDropCount() const {
    return connection_drop_count_;
}

uint32_t ConnectionManager::getRecoveryAttempts() const {
    return recovery_attempts_;
}

ConnectionManager::RecoveryStrategy ConnectionManager::getCurrentStrategy() const {
    return current_strategy_;
}

void ConnectionManager::resetErrorCounters() {
    connection_drop_count_ = 0;
    recovery_attempts_ = 0;
    consecutive_failures_ = 0;
    consecutive_ping_failures_ = 0;
    current_strategy_ = RETRY;
    current_backoff_ms_ = base_backoff_ms_;
}

void ConnectionManager::setMaxRetries(uint8_t max_retries) {
    max_retries_ = max_retries;
}

void ConnectionManager::setBaseBackoffMs(uint32_t base_ms) {
    base_backoff_ms_ = base_ms;
    if (current_backoff_ms_ < base_ms) {
        current_backoff_ms_ = base_ms;
    }
}

void ConnectionManager::setMaxBackoffMs(uint32_t max_ms) {
    max_backoff_ms_ = max_ms;
}

void ConnectionManager::setPingInterval(uint32_t interval_ms) {
    ping_interval_ms_ = interval_ms;
}

bool ConnectionManager::isSafeModeActive() const {
    return safe_mode_active_;
}

void ConnectionManager::enterSafeMode() {
    if (!safe_mode_active_) {
        safe_mode_active_ = true;
        safe_mode_start_time_ = millis();
    }
}

void ConnectionManager::exitSafeMode() {
    safe_mode_active_ = false;
    safe_mode_start_time_ = 0;
}

void ConnectionManager::transitionToState(State new_state) {
    if (new_state != current_state_) {
        logStateChange(current_state_, new_state);
        
        previous_state_ = current_state_;
        current_state_ = new_state;
        state_change_time_ = millis();
        
        // Handle state-specific initialization
        switch (new_state) {
            case CONNECTED:
                connection_start_time_ = millis();
                consecutive_failures_ = 0;
                exitSafeMode(); // Exit safe mode on successful connection
                break;
                
            case DISCONNECTED:
                connection_start_time_ = 0;
                enterSafeMode(); // Enter safe mode on disconnection
                break;
                
            case ERROR:
                updateRecoveryStrategy();
                enterSafeMode();
                break;
                
            case WAITING:
                calculateBackoff();
                break;
                
            default:
                break;
        }
    }
}

void ConnectionManager::updateConnectionHealth() {
    // This method can be extended to include additional health metrics
    // such as message latency, throughput, etc.
}

void ConnectionManager::handleConnectionFailure() {
    consecutive_failures_++;
    
    // Update recovery strategy based on failure count
    updateRecoveryStrategy();
    
    // Calculate new backoff time
    calculateBackoff();
}

void ConnectionManager::calculateBackoff() {
    // Exponential backoff with jitter
    uint32_t exponential_backoff = base_backoff_ms_ * (1 << std::min(consecutive_failures_, (uint32_t)8));
    
    // Add jitter (±25%)
    uint32_t jitter = exponential_backoff / 4;
    uint32_t random_jitter = random(0, jitter * 2) - jitter;
    
    current_backoff_ms_ = std::min(exponential_backoff + random_jitter, max_backoff_ms_);
}

bool ConnectionManager::shouldRetry() const {
    uint32_t now = millis();
    
    // Check if enough time has passed since last retry
    if (now - last_retry_time_ < current_backoff_ms_) {
        return false;
    }
    
    // Check retry limits based on strategy
    switch (current_strategy_) {
        case RETRY:
            return recovery_attempts_ < max_retries_;
            
        case RESET:
            return recovery_attempts_ < max_retries_ * 2;
            
        case RESTART:
            return recovery_attempts_ < max_retries_ * 3;
            
        default:
            return false;
    }
}

void ConnectionManager::logStateChange(State old_state, State new_state) {
    // This would integrate with the debug system
    // For now, we'll keep it simple since we can't use Serial
    // The debug manager will handle actual logging
}

void ConnectionManager::updateRecoveryStrategy() {
    if (consecutive_failures_ < 3) {
        current_strategy_ = RETRY;
    } else if (consecutive_failures_ < 6) {
        current_strategy_ = RESET;
    } else {
        current_strategy_ = RESTART;
    }
}