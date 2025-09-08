#ifndef INTER_CORE_COMM_H
#define INTER_CORE_COMM_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "system_config.h"
#include "mutex_wrapper.h"
#include "time_utils.h"
#include "validation_utils.h"
#include "error_handler.h"
#include "initializable.h"

// =============================================================================
// THREAD-SAFE CAR STATE CLASS
// =============================================================================

class ThreadSafeCarState : public Initializable {
public:
    // Constructor and destructor
    ThreadSafeCarState();
    ~ThreadSafeCarState();
    
    // Initialization and cleanup
    bool initialize();
    void cleanup();
    
    // Command operations (typically called from ROS core)
    bool setCommand(const MotorCommand_t& cmd);
    bool getCommand(MotorCommand_t& cmd);
    bool hasNewCommand();
    void clearNewCommandFlag();
    
    // Status operations (typically called from Motor Control core)
    bool setStatus(const MotorStatus_t& status);
    bool getStatus(MotorStatus_t& status);
    
    // Queue operations
    bool sendCommandToQueue(const MotorCommand_t& cmd);
    bool receiveCommandFromQueue(MotorCommand_t& cmd, TickType_t timeout = 0);
    bool sendStatusToQueue(const MotorStatus_t& status);
    bool receiveStatusFromQueue(MotorStatus_t& status, TickType_t timeout = 0);
    
    // Enhanced queue operations with priority
    bool sendCommandToQueueFromISR(const MotorCommand_t& cmd, BaseType_t* pxHigherPriorityTaskWoken = nullptr);
    bool sendStatusToQueueFromISR(const MotorStatus_t& status, BaseType_t* pxHigherPriorityTaskWoken = nullptr);
    bool receiveCommandFromQueueFromISR(MotorCommand_t& cmd, BaseType_t* pxHigherPriorityTaskWoken = nullptr);
    bool receiveStatusFromQueueFromISR(MotorStatus_t& status, BaseType_t* pxHigherPriorityTaskWoken = nullptr);
    
    // Queue management
    void clearCommandQueue();
    void clearStatusQueue();
    bool isCommandQueueFull();
    bool isStatusQueueFull();
    
    // Queue status information
    uint32_t getCommandQueueWaiting();
    uint32_t getStatusQueueWaiting();
    uint32_t getCommandQueueSpacesAvailable();
    uint32_t getStatusQueueSpacesAvailable();
    
    // Error handling and statistics
    uint32_t getCommandQueueOverruns();
    uint32_t getStatusQueueOverruns();
    uint32_t getMutexTimeouts();
    void resetErrorCounters();
    
    // Health check
    bool isHealthy() const;
    
    // Detailed statistics
    struct QueueStatistics {
        uint32_t waiting_messages;
        uint32_t available_spaces;
        uint32_t overruns;
        uint32_t utilization_percent;
        bool is_full;
    };
    
    QueueStatistics getCommandQueueStatistics();
    QueueStatistics getStatusQueueStatistics();
    
protected:
    // Virtual method implementations from Initializable
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    
private:
    // Mutex handles for thread-safe access
    MutexWrapper commandMutex;
    MutexWrapper statusMutex;
    MutexWrapper errorCounterMutex;
    
    // Queue handles for inter-core communication
    QueueHandle_t commandQueue;
    QueueHandle_t statusQueue;
    
    // Current command and status data
    MotorCommand_t currentCommand;
    MotorStatus_t currentStatus;
    
    // State flags
    volatile bool newCommandAvailable;
    
    // Error counters
    volatile uint32_t commandQueueOverruns;
    volatile uint32_t statusQueueOverruns;
    volatile uint32_t mutexTimeouts;
    
    // Configuration
    SystemConfig_t config;
    
    // Queue overflow handling
    bool handleCommandQueueOverflow(const MotorCommand_t& cmd);
    bool handleStatusQueueOverflow(const MotorStatus_t& status);
};

// =============================================================================
// INTER-CORE COMMUNICATION MANAGER
// =============================================================================

class InterCoreCommunication : public Initializable {
public:
    // Singleton pattern
    static InterCoreCommunication& getInstance();
    
    // Initialization and cleanup
    bool initialize(const SystemConfig_t& config = DEFAULT_SYSTEM_CONFIG);
    void cleanup();
    
    // Access to thread-safe car state
    ThreadSafeCarState& getCarState();
    
    // Performance monitoring
    void updatePerformanceMetrics(PerformanceMetrics_t& metrics);
    
    // Health monitoring
    bool isHealthy() const;
    
    // Error reporting
    void reportError(uint8_t errorFlag, const char* description = nullptr);
    uint8_t getErrorFlags();
    void clearErrorFlags();
    
protected:
    // Virtual method implementations from Initializable
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    
private:
    // Private constructor for singleton
    InterCoreCommunication();
    ~InterCoreCommunication();
    
    // Prevent copying
    InterCoreCommunication(const InterCoreCommunication&) = delete;
    InterCoreCommunication& operator=(const InterCoreCommunication&) = delete;
    
    // Member variables
    ThreadSafeCarState carState;
    SystemConfig_t systemConfig;
    volatile uint8_t errorFlags;
    
    // Error reporting mutex
    MutexWrapper errorMutex;
};

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Copy command with validation
bool copyMotorCommand(const MotorCommand_t& src, MotorCommand_t& dst);

// Copy status with validation
bool copyMotorStatus(const MotorStatus_t& src, MotorStatus_t& dst);

// Calculate queue utilization percentage
uint8_t calculateQueueUtilization(QueueHandle_t queue);

#endif // INTER_CORE_COMM_H