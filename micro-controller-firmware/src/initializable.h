#ifndef INITIALIZABLE_H
#define INITIALIZABLE_H

#include <stdbool.h>
#include "error_handler.h"

// =============================================================================
// COMMON INITIALIZABLE BASE CLASS
// =============================================================================

class Initializable {
public:
    Initializable(const char* componentName);
    virtual ~Initializable() = default;
    
    // Core initialization interface
    bool initialize();
    void cleanup();
    bool isInitialized() const { return initialized; }
    
    // Health checking
    bool isHealthy() const;
    
    // Error handling
    void reportInitializationError(const char* error);
    uint32_t getInitializationErrors() const { return initializationErrors; }
    
protected:
    // Virtual methods to be implemented by derived classes
    virtual bool doInitialize() = 0;
    virtual void doCleanup() = 0;
    virtual bool doHealthCheck() const { return true; }
    
    // Helper methods
    void setInitialized(bool state) { initialized = state; }
    void incrementErrorCount() { initializationErrors++; }
    
protected:
    ErrorHandler errorHandler;
    
private:
    bool initialized;
    uint32_t initializationErrors;
};

// =============================================================================
// TASK-BASED INITIALIZABLE
// =============================================================================

class TaskInitializable : public Initializable {
public:
    TaskInitializable(const char* componentName);
    virtual ~TaskInitializable() = default;
    
    // Task management
    bool startTask();
    void stopTask();
    bool isTaskRunning() const { return taskRunning; }
    
    // Task health checking
    bool isTaskHealthy() const;
    
protected:
    // Virtual methods to be implemented by derived classes
    virtual bool doInitialize() override;
    virtual void doCleanup() override;
    virtual bool doHealthCheck() const override;
    
    virtual bool createTask() = 0;
    virtual void destroyTask() = 0;
    virtual void taskFunction() = 0;
    
    // Static task function wrapper
    static void staticTaskFunction(void* parameter);
    
    // Task management
    void setTaskRunning(bool state) { taskRunning = state; }
    
protected:
    TaskErrorHandler taskErrorHandler;
    TaskHandle_t taskHandle;
    
private:
    bool taskRunning;
};

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

#define INITIALIZE_COMPONENT(component) \
    if (!component.initialize()) { \
        component.reportInitializationError("Failed to initialize"); \
        return false; \
    }

#define CLEANUP_COMPONENT(component) \
    component.cleanup();

#define CHECK_COMPONENT_HEALTH(component) \
    if (!component.isHealthy()) { \
        component.reportInitializationError("Component unhealthy"); \
        return false; \
    }

#endif // INITIALIZABLE_H
