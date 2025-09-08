#include "initializable.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =============================================================================
// INITIALIZABLE IMPLEMENTATION
// =============================================================================

Initializable::Initializable(const char* componentName)
    : initialized(false), initializationErrors(0), errorHandler(componentName) {
}

bool Initializable::initialize() {
    if (initialized) {
        return true; // Already initialized
    }
    
    if (doInitialize()) {
        initialized = true;
        return true;
    } else {
        incrementErrorCount();
        return false;
    }
}

void Initializable::cleanup() {
    if (initialized) {
        doCleanup();
        initialized = false;
    }
}

bool Initializable::isHealthy() const {
    if (!initialized) return false;
    return doHealthCheck() && errorHandler.isHealthy();
}

void Initializable::reportInitializationError(const char* error) {
    incrementErrorCount();
    errorHandler.handleError(error);
}

// =============================================================================
// TASK INITIALIZABLE IMPLEMENTATION
// =============================================================================

TaskInitializable::TaskInitializable(const char* componentName)
    : Initializable(componentName), taskRunning(false), taskErrorHandler(componentName) {
}

bool TaskInitializable::doInitialize() {
    return createTask();
}

void TaskInitializable::doCleanup() {
    stopTask();
    destroyTask();
}

bool TaskInitializable::doHealthCheck() const {
    return isTaskHealthy();
}

bool TaskInitializable::startTask() {
    if (!isInitialized() || taskRunning) {
        return false;
    }
    
    // Task creation is handled in doInitialize()
    // This method can be used for additional task startup logic
    setTaskRunning(true);
    return true;
}

void TaskInitializable::stopTask() {
    if (taskRunning) {
        setTaskRunning(false);
        // Task destruction is handled in doCleanup()
    }
}

bool TaskInitializable::isTaskHealthy() const {
    return taskErrorHandler.isTaskHealthy();
}

void TaskInitializable::staticTaskFunction(void* parameter) {
    TaskInitializable* instance = static_cast<TaskInitializable*>(parameter);
    if (instance != nullptr) {
        instance->taskFunction();
    }
}
