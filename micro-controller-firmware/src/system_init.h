#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "system_config.h"
#include "inter_core_comm.h"
#include "car.h"
#include "ros_comm_task.h"
#include "ros_logger.h"
#include "error_handler.h"
#include "initializable.h"

// =============================================================================
// SYSTEM INITIALIZATION CLASS
// =============================================================================

class SystemInitializer : public Initializable {
public:
    // Constructor and destructor
    SystemInitializer();
    ~SystemInitializer();
    
    // Main initialization function
    bool initializeSystem(const SystemConfig_t& config = DEFAULT_SYSTEM_CONFIG);
    void shutdownSystem();
    
    // Individual component initialization
    bool initializeUSB();
    bool initializeInterCoreCommunication();
    bool initializeMotorControlTask();
    bool initializeROSCommunicationTask();
    
    // System health and status
    bool isSystemHealthy();
    bool isSystemInitialized();
    void printSystemStatus();
    
    // Error handling
    uint32_t getInitializationErrors();
    void clearInitializationErrors();
    void reportInitializationError(const char* component, const char* error); // Public for testing
    
    // Access to system components
    MultiCoreCar* getMultiCoreCar();
    ROSCommTask* getROSCommTask();
    InterCoreCommunication* getInterCoreCommunication();
    
protected:
    // Virtual method implementations from Initializable
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    
private:
    // System components
    MultiCoreCar* multiCoreCar;
    ROSCommTask* rosCommTask;
    InterCoreCommunication* interCoreComm;
    
    // System state
    bool usbInitialized;
    bool interCoreInitialized;
    bool motorTaskInitialized;
    bool rosTaskInitialized;
    
    // Configuration
    SystemConfig_t systemConfig;
    
    // Private helper methods
    bool validateConfiguration(const SystemConfig_t& config);
    bool checkTaskCreation(BaseType_t result, const char* taskName);
    void printTaskInfo(const char* taskName, uint8_t priority, uint8_t coreId, uint32_t stackSize);
    
    // Error codes
    enum ErrorCode {
        ERROR_USB_INIT_FAILED = (1 << 0),
        ERROR_INTER_CORE_INIT_FAILED = (1 << 1),
        ERROR_MOTOR_TASK_INIT_FAILED = (1 << 2),
        ERROR_ROS_TASK_INIT_FAILED = (1 << 3),
        ERROR_CONFIG_VALIDATION_FAILED = (1 << 4),
        ERROR_TASK_CREATION_FAILED = (1 << 5),
        ERROR_MEMORY_ALLOCATION_FAILED = (1 << 6)
    };
};

// =============================================================================
// GLOBAL SYSTEM INITIALIZER INSTANCE
// =============================================================================

// Global system initializer instance
extern SystemInitializer systemInitializer;

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Initialize the entire system
bool initializeSystem(const SystemConfig_t& config = DEFAULT_SYSTEM_CONFIG);

// Shutdown the entire system
void shutdownSystem();

// Get system health status
bool isSystemHealthy();

// Print comprehensive system status
void printSystemStatus();

// Get system component references
MultiCoreCar* getMultiCoreCar();
ROSCommTask* getROSCommTask();
InterCoreCommunication* getInterCoreCommunication();

#endif // SYSTEM_INIT_H
