#ifndef RECOVERY_MANAGER_H
#define RECOVERY_MANAGER_H

#include <stdint.h>
#include "connection_manager.h"

class Car; // Forward declaration

class RecoveryManager {
public:
    enum SafeModeLevel {
        NORMAL_OPERATION,
        DEGRADED_CONNECTIVITY,
        CONNECTION_LOST,
        CRITICAL_ERROR
    };

    struct RecoveryMetrics {
        uint32_t total_recovery_attempts;
        uint32_t successful_recoveries;
        uint32_t failed_recoveries;
        uint32_t safe_mode_activations;
        uint32_t emergency_stops;
        uint32_t last_recovery_time;
        float recovery_success_rate;
    };

    RecoveryManager(ConnectionManager* conn_mgr, Car* car_instance);
    
    // Core recovery functionality
    void update();
    bool attemptRecovery();
    void handleConnectionDrop();
    void handleCriticalError();
    
    // Safe mode operations
    void enterSafeMode(SafeModeLevel level);
    void exitSafeMode();
    SafeModeLevel getCurrentSafeModeLevel() const;
    bool isSafeModeActive() const;
    
    // Emergency procedures
    void emergencyStop();
    void safeMotorStop();
    bool isEmergencyStopActive() const;
    
    // Recovery strategies
    bool executeGraduatedRecovery();
    bool executeConnectionReset();
    bool executeSystemRestart();
    
    // State persistence
    void saveSystemState();
    bool restoreSystemState();
    void clearPersistedState();
    
    // Monitoring and metrics
    RecoveryMetrics getMetrics() const;
    void resetMetrics();
    uint32_t getTimeSinceLastRecovery() const;
    
    // Configuration
    void setRecoveryTimeout(uint32_t timeout_ms);
    void setMaxRecoveryAttempts(uint8_t max_attempts);
    void setEmergencyStopTimeout(uint32_t timeout_ms);

private:
    // Dependencies
    ConnectionManager* connection_manager_;
    Car* car_;
    
    // Safe mode state
    SafeModeLevel current_safe_mode_;
    uint32_t safe_mode_start_time_;
    bool emergency_stop_active_;
    uint32_t emergency_stop_time_;
    
    // Recovery state
    bool recovery_in_progress_;
    uint32_t recovery_start_time_;
    uint32_t recovery_timeout_ms_;
    uint8_t max_recovery_attempts_;
    uint8_t current_recovery_attempt_;
    
    // Metrics
    RecoveryMetrics metrics_;
    
    // Persisted state
    struct PersistedState {
        float last_speed;
        float last_steering_angle;
        uint32_t timestamp;
        bool valid;
    } persisted_state_;
    
    // Configuration
    uint32_t emergency_stop_timeout_ms_;
    
    // Internal methods
    void updateSafeModeOperations();
    void monitorSystemHealth();
    bool isRecoveryTimeout() const;
    void logRecoveryEvent(const char* event, bool success);
    void updateMetrics(bool recovery_success);
    SafeModeLevel determineSafeModeLevel() const;
    void applySafeModeRestrictions();
    bool validateSystemState() const;
};

#endif // RECOVERY_MANAGER_H