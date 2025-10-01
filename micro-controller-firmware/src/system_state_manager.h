#ifndef SYSTEM_STATE_MANAGER_H
#define SYSTEM_STATE_MANAGER_H

#include <stdint.h>

class ConnectionManager;
class RecoveryManager;
class Car;

class SystemStateManager {
public:
    enum OperationalMode {
        FULL_OPERATION,
        DEGRADED_OPERATION,
        SAFE_MODE,
        EMERGENCY_MODE
    };

    enum SystemCapability {
        MOTOR_CONTROL = 0x01,
        STEERING_CONTROL = 0x02,
        SENSOR_READING = 0x04,
        ROS_COMMUNICATION = 0x08,
        ODOMETRY_PUBLISHING = 0x10,
        DEBUG_LOGGING = 0x20
    };

    struct SystemState {
        OperationalMode mode;
        uint8_t available_capabilities;
        uint32_t mode_start_time;
        uint32_t last_full_operation_time;
        bool persistent_state_valid;
        float last_speed_command;
        float last_steering_command;
        uint32_t command_timestamp;
        uint32_t uptime_ms;
        bool initialization_complete;
    };

    struct DegradationConfig {
        uint32_t connection_loss_timeout_ms;
        uint32_t degraded_mode_timeout_ms;
        float max_degraded_speed;
        float max_degraded_steering_angle;
        bool allow_cached_operation;
        uint32_t cache_validity_ms;
    };

    SystemStateManager(ConnectionManager* conn_mgr, RecoveryManager* recovery_mgr, Car* car_instance);
    
    // Core functionality
    void update();
    void transitionToMode(OperationalMode new_mode);
    OperationalMode getCurrentMode() const;
    
    // Capability management
    bool isCapabilityAvailable(SystemCapability capability) const;
    void enableCapability(SystemCapability capability);
    void disableCapability(SystemCapability capability);
    uint8_t getAvailableCapabilities() const;
    
    // State persistence
    void persistCurrentState();
    bool restorePersistedState();
    void clearPersistedState();
    bool hasValidPersistedState() const;
    
    // Graceful degradation
    void handleConnectionLoss();
    void handlePartialFailure(SystemCapability failed_capability);
    void handleSystemRecovery();
    
    // Fallback operations
    bool executeFallbackOperation();
    void maintainMinimalOperation();
    void executeEmergencyProtocol();
    
    // Configuration
    void setDegradationConfig(const DegradationConfig& config);
    DegradationConfig getDegradationConfig() const;
    
    // Monitoring
    SystemState getSystemState() const;
    uint32_t getTimeInCurrentMode() const;
    uint32_t getTimeSinceFullOperation() const;
    bool isSystemHealthy() const;
    
    // Command processing with degradation
    bool processSpeedCommand(float speed);
    bool processSteeringCommand(float angle);
    bool canAcceptCommands() const;

private:
    // Dependencies
    ConnectionManager* connection_manager_;
    RecoveryManager* recovery_manager_;
    Car* car_;
    
    // Current state
    SystemState current_state_;
    DegradationConfig config_;
    
    // Cached commands for fallback operation
    struct CachedCommand {
        float speed;
        float steering_angle;
        uint32_t timestamp;
        bool valid;
    } last_valid_command_;
    
    // Internal methods
    void updateOperationalMode();
    void updateCapabilities();
    void applyModeRestrictions();
    bool shouldTransitionToMode(OperationalMode mode) const;
    void logModeTransition(OperationalMode old_mode, OperationalMode new_mode);
    void initializeMode(OperationalMode mode);
    void cleanupMode(OperationalMode mode);
    bool validateCommand(float speed, float steering_angle) const;
    void cacheValidCommand(float speed, float steering_angle);
    bool isCommandCacheValid() const;
    void executeMinimalSafetyChecks();
};

#endif // SYSTEM_STATE_MANAGER_H