#ifndef CONNECTION_MANAGER_H
#define CONNECTION_MANAGER_H

#include <stdint.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

class ConnectionManager {
public:
    enum State {
        WAITING,
        AVAILABLE,
        CONNECTED,
        DISCONNECTED,
        ERROR
    };

    enum RecoveryStrategy {
        RETRY,
        RESET,
        RESTART
    };

    ConnectionManager();
    
    // Core functionality
    void update();
    State getState() const;
    bool isHealthy() const;
    void forceReconnect();
    
    // Health monitoring
    bool pingAgent(uint32_t timeout_ms = 100);
    uint32_t getConnectionUptime() const;
    uint32_t getLastPingTime() const;
    float getConnectionQuality() const;
    
    // Error handling and recovery
    uint32_t getConnectionDropCount() const;
    uint32_t getRecoveryAttempts() const;
    RecoveryStrategy getCurrentStrategy() const;
    void resetErrorCounters();
    
    // Configuration
    void setMaxRetries(uint8_t max_retries);
    void setBaseBackoffMs(uint32_t base_ms);
    void setMaxBackoffMs(uint32_t max_ms);
    void setPingInterval(uint32_t interval_ms);
    
    // Safe mode operations
    bool isSafeModeActive() const;
    void enterSafeMode();
    void exitSafeMode();

private:
    // State management
    State current_state_;
    State previous_state_;
    uint32_t state_change_time_;
    
    // Connection health
    uint32_t connection_start_time_;
    uint32_t last_ping_time_;
    uint32_t last_successful_ping_;
    uint32_t ping_interval_ms_;
    uint32_t consecutive_ping_failures_;
    uint32_t total_ping_attempts_;
    uint32_t successful_pings_;
    
    // Error tracking and recovery
    uint32_t connection_drop_count_;
    uint32_t recovery_attempts_;
    uint32_t consecutive_failures_;
    RecoveryStrategy current_strategy_;
    
    // Exponential backoff
    uint8_t max_retries_;
    uint32_t base_backoff_ms_;
    uint32_t max_backoff_ms_;
    uint32_t current_backoff_ms_;
    uint32_t last_retry_time_;
    
    // Safe mode
    bool safe_mode_active_;
    uint32_t safe_mode_start_time_;
    
    // Internal methods
    void transitionToState(State new_state);
    void updateConnectionHealth();
    void handleConnectionFailure();
    void calculateBackoff();
    bool shouldRetry() const;
    void logStateChange(State old_state, State new_state);
    void updateRecoveryStrategy();
};

#endif // CONNECTION_MANAGER_H