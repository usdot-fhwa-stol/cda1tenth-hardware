#include "system_init.h"
#include "USB.h"
#include "USBCDC.h"
#include <micro_ros_platformio.h>
#include <stdio.h>
#include "neopixel_led.h"
#include "web_debug_server.h"

// External USB Serial declaration
extern USBCDC USBSerial;

// Global system initializer instance
SystemInitializer systemInitializer;

// =============================================================================
// SYSTEM INITIALIZER IMPLEMENTATION
// =============================================================================

SystemInitializer::SystemInitializer() 
    : Initializable("SystemInitializer"),
      multiCoreCar(nullptr), rosCommTask(nullptr), interCoreComm(nullptr),
      usbInitialized(false), interCoreInitialized(false),
      motorTaskInitialized(false), rosTaskInitialized(false) {
    
    systemConfig = DEFAULT_SYSTEM_CONFIG;
}

SystemInitializer::~SystemInitializer() {
    shutdownSystem();
}

bool SystemInitializer::initializeSystem(const SystemConfig_t& config) {
    // Validate configuration
    if (!validateConfiguration(config)) {
        errorHandler.handleError("Invalid configuration parameters");
        return false;
    }
    
    systemConfig = config;
    
    // Use base class initialization
    return Initializable::initialize();
}

void SystemInitializer::shutdownSystem() {
    Initializable::cleanup();
}

// Virtual method implementations from Initializable
bool SystemInitializer::doInitialize() {
    // Initialize components in order
    if (!initializeUSB()) {
        return false;
    }
    
    if (!initializeInterCoreCommunication()) {
        return false;
    }
    
    if (!initializeMotorControlTask()) {
        return false;
    }
    
    if (!initializeROSCommunicationTask()) {
        return false;
    }
    
    // Print system status
    printSystemStatus();
    
    return true;
}

void SystemInitializer::doCleanup() {
    // Clean up components in reverse order
    if (rosTaskInitialized && rosCommTask != nullptr) {
        rosCommTask->stopTask();
        rosCommTask->cleanup();
        delete rosCommTask;
        rosCommTask = nullptr;
        rosTaskInitialized = false;
    }
    
    if (motorTaskInitialized && multiCoreCar != nullptr) {
        multiCoreCar->cleanup();
        delete multiCoreCar;
        multiCoreCar = nullptr;
        motorTaskInitialized = false;
    }
    
    if (interCoreInitialized && interCoreComm != nullptr) {
        interCoreComm->cleanup();
        interCoreComm = nullptr;
        interCoreInitialized = false;
    }
    
    if (usbInitialized) {
        // USB cleanup if needed
        usbInitialized = false;
    }
}

bool SystemInitializer::doHealthCheck() const {
    // Check if all components are healthy
    if (interCoreInitialized && interCoreComm != nullptr && !interCoreComm->isHealthy()) {
        return false;
    }
    
    if (motorTaskInitialized && multiCoreCar != nullptr && !multiCoreCar->isHealthy()) {
        return false;
    }
    
    if (rosTaskInitialized && rosCommTask != nullptr && !rosCommTask->isHealthy()) {
        return false;
    }
    
    return true;
}

bool SystemInitializer::initializeUSB() {
    if (usbInitialized) {
        return true;
    }
    
    // Initialize NeoPixel LED for debugging
    if (!statusLED.initialize()) {
        errorHandler.handleError("Failed to initialize NeoPixel LED");
        return false;
    }
    statusLED.showUSBInit(); // Blue LED = USB init starting
    
    // Add delay to show LED state
    delay(1000);
    
    // Initialize web debug server first for immediate debug access
    statusLED.setColor(255, 0, 255); // Magenta = web server init
    delay(500);
    
    if (!webDebugServer.initialize()) {
        statusLED.setColor(255, 0, 0); // Red = web server failed
        delay(2000);
        errorHandler.handleError("Failed to initialize web debug server");
        return false;
    }
    
    // Start WiFi access point
    statusLED.setColor(255, 255, 0); // Yellow = WiFi starting
    delay(500);
    webDebugServer.startWiFiAP();
    
    // Start web server
    statusLED.setColor(0, 255, 255); // Cyan = web server starting
    delay(500);
    webDebugServer.startServer();
    
    // Log initial status
    webDebugServer.logInfo("SYSTEM", "Web debug server initialized");
    webDebugServer.updateSystemStatus("Web interface ready, starting USB initialization...");
    
    // Now initialize USB CDC (simplified approach)
    statusLED.setColor(0, 255, 0); // Green = USB starting
    delay(500);
    webDebugServer.logInfo("USB", "Starting USB CDC initialization");
    
    USB.begin();
    USBSerial.begin(systemConfig.serial_baud_rate);
    
    // Simple wait with timeout to prevent infinite loop
    uint32_t startTime = millis();
    while (!USBSerial && (millis() - startTime) < 5000) {
        delay(50);
    }
    
    // Check if USB is ready
    if (!USBSerial) {
        statusLED.setColor(255, 165, 0); // Orange = USB failed but continuing
        delay(1000);
        webDebugServer.logWarning("USB", "USB CDC not ready after 5s, continuing anyway");
        // Don't return false - continue with web interface only
    } else {
        statusLED.setColor(0, 255, 0); // Green = USB success
        delay(500);
        webDebugServer.logInfo("USB", "USB CDC initialized successfully");
    }
    
    delay(2000); // Give more time for serial to initialize
    
    // Test serial communication only if USB is available
    if (USBSerial) {
        USBSerial.println("USB CDC initialized successfully");
        USBSerial.flush();
        
        // Configure Micro-ROS library to use USB CDC serial
        set_microros_serial_transports(USBSerial);
        webDebugServer.logInfo("USB", "Micro-ROS transport configured");
    } else {
        webDebugServer.logWarning("USB", "USB not available, micro-ROS transport not configured");
    }
    
    // LED sequence = USB ready, micro-ROS transport configured
    statusLED.showUSBReady();
    
    // Update web debug status
    webDebugServer.logInfo("USB", "USB CDC initialized successfully");
    webDebugServer.updateSystemStatus("System fully initialized - USB and web interface ready");
    
    usbInitialized = true;
    
    return true;
}

bool SystemInitializer::initializeInterCoreCommunication() {
    if (interCoreInitialized) {
        return true;
    }
    
    // Get inter-core communication instance
    interCoreComm = &InterCoreCommunication::getInstance();
    
    // Initialize inter-core communication
    if (!interCoreComm->initialize(systemConfig)) {
        errorHandler.handleError("Failed to initialize inter-core communication");
        return false;
    }
    
    interCoreInitialized = true;
    // Note: Can't use ROS logger here as it's not initialized yet
    
    return true;
}

bool SystemInitializer::initializeMotorControlTask() {
    if (motorTaskInitialized) {
        return true;
    }
    
    // Create multi-core car instance
    multiCoreCar = new MultiCoreCar(CS_RIGHT, CS_LEFT, CS_STEER);
    if (multiCoreCar == nullptr) {
        errorHandler.handleError("Failed to allocate memory for MultiCoreCar");
        return false;
    }
    
    // Set system configuration
    multiCoreCar->setSystemConfig(systemConfig);
    
    // Initialize multi-core car system
    if (!multiCoreCar->initialize()) {
        errorHandler.handleError("Failed to initialize multi-core car system");
        delete multiCoreCar;
        multiCoreCar = nullptr;
        return false;
    }
    
    motorTaskInitialized = true;
    
    // Log task information (will be logged once ROS logger is available)
    // printTaskInfo("MotorControlTask", 
    //               systemConfig.motor_task_priority, 
    //               systemConfig.motor_task_core_id, 
    //               systemConfig.motor_task_stack_size);
    
    return true;
}

bool SystemInitializer::initializeROSCommunicationTask() {
    if (rosTaskInitialized) {
        return true;
    }
    
    // Create ROS communication task instance
    rosCommTask = new ROSCommTask();
    if (rosCommTask == nullptr) {
        errorHandler.handleError("Failed to allocate memory for ROSCommTask");
        return false;
    }
    
    // Get car state reference
    ThreadSafeCarState& carState = interCoreComm->getCarState();
    
    // Initialize ROS communication task
    if (!rosCommTask->initialize(&carState, systemConfig)) {
        errorHandler.handleError("Failed to initialize ROS communication task");
        delete rosCommTask;
        rosCommTask = nullptr;
        return false;
    }
    
    // Start ROS communication task
    if (!rosCommTask->startTask()) {
        errorHandler.handleError("Failed to start ROS communication task");
        rosCommTask->cleanup();
        delete rosCommTask;
        rosCommTask = nullptr;
        return false;
    }
    
    // Log successful task start
    webDebugServer.logInfo("SYSTEM", "ROS communication task started successfully");
    
    rosTaskInitialized = true;
    
    // Log task information via ROS logger (now available)
    ROS_LOG_INFOF("SYSTEM", "Task Created: ROSCommTask | Core: %d | Priority: %d | Stack: %d bytes",
                  systemConfig.ros_task_core_id, systemConfig.ros_task_priority, ROS_COMM_TASK_STACK_SIZE);
    
    ROS_LOG_INFO("SYSTEM", "ROS communication task initialized successfully");
    
    return true;
}

bool SystemInitializer::isSystemHealthy() {
    return Initializable::isHealthy();
}

bool SystemInitializer::isSystemInitialized() {
    return isInitialized();
}

void SystemInitializer::printSystemStatus() {
    // Use ROS logger to log system status
    ROS_LOG_INFO("SYSTEM", "=== SYSTEM STATUS ===");
    ROS_LOG_INFOF("SYSTEM", "System Initialized: %s", isInitialized() ? "YES" : "NO");
    ROS_LOG_INFOF("SYSTEM", "USB CDC: %s", usbInitialized ? "OK" : "FAIL");
    ROS_LOG_INFOF("SYSTEM", "Inter-Core Comm: %s", interCoreInitialized ? "OK" : "FAIL");
    ROS_LOG_INFOF("SYSTEM", "Motor Control Task: %s", motorTaskInitialized ? "OK" : "FAIL");
    ROS_LOG_INFOF("SYSTEM", "ROS Comm Task: %s", rosTaskInitialized ? "OK" : "FAIL");
    ROS_LOG_INFOF("SYSTEM", "System Health: %s", isSystemHealthy() ? "HEALTHY" : "UNHEALTHY");
    
    if (getInitializationErrors() > 0) {
        ROS_LOG_ERRORF("SYSTEM", "Initialization Errors: 0x%08X", getInitializationErrors());
    }
    
    // Log task details
    if (motorTaskInitialized && multiCoreCar != nullptr) {
        ROS_LOG_INFOF("SYSTEM", "Motor Task - Core: %d, Priority: %d, Stack: %d bytes",
                      systemConfig.motor_task_core_id,
                      systemConfig.motor_task_priority,
                      systemConfig.motor_task_stack_size);
    }
    
    if (rosTaskInitialized && rosCommTask != nullptr) {
        ROS_LOG_INFOF("SYSTEM", "ROS Task - Core: %d, Priority: %d, Stack: %d bytes",
                      systemConfig.ros_task_core_id,
                      systemConfig.ros_task_priority,
                      ROS_COMM_TASK_STACK_SIZE);
    }
    
    ROS_LOG_INFO("SYSTEM", "====================");
}

uint32_t SystemInitializer::getInitializationErrors() {
    return errorHandler.getErrorCount();
}

void SystemInitializer::clearInitializationErrors() {
    errorHandler.resetErrorCount();
}

MultiCoreCar* SystemInitializer::getMultiCoreCar() {
    return multiCoreCar;
}

ROSCommTask* SystemInitializer::getROSCommTask() {
    return rosCommTask;
}

InterCoreCommunication* SystemInitializer::getInterCoreCommunication() {
    return interCoreComm;
}

bool SystemInitializer::validateConfiguration(const SystemConfig_t& config) {
    // Validate task priorities
    if (config.motor_task_priority < 1 || config.motor_task_priority > 25) {
        return false;
    }
    if (config.ros_task_priority < 1 || config.ros_task_priority > 25) {
        return false;
    }
    
    // Validate core IDs
    if (config.motor_task_core_id > 1 || config.ros_task_core_id > 1) {
        return false;
    }
    
    // Validate stack sizes
    if (config.motor_task_stack_size < 1024 || config.motor_task_stack_size > 32768) {
        return false;
    }
    
    // Validate timing parameters
    if (config.motor_control_frequency_hz < 100 || config.motor_control_frequency_hz > 10000) {
        return false;
    }
    
    if (config.ros_spin_timeout_ms < 1 || config.ros_spin_timeout_ms > 1000) {
        return false;
    }
    
    // Validate queue sizes
    if (config.command_queue_size < 1 || config.command_queue_size > 32) {
        return false;
    }
    if (config.status_queue_size < 1 || config.status_queue_size > 32) {
        return false;
    }
    
    return true;
}


bool SystemInitializer::checkTaskCreation(BaseType_t result, const char* taskName) {
    if (result != pdPASS) {
        errorHandler.handleError("Task creation failed");
        return false;
    }
    return true;
}

void SystemInitializer::printTaskInfo(const char* taskName, uint8_t priority, uint8_t coreId, uint32_t stackSize) {
    // Use ROS logger if available, otherwise fall back to USB
    if (rosLogger.isInitialized()) {
        ROS_LOG_INFOF("TASK", "Task Created: %s | Core: %d | Priority: %d | Stack: %d bytes",
                      taskName, coreId, priority, stackSize);
    } else if (usbInitialized) {
        USBSerial.printf("Task Created: %s | Core: %d | Priority: %d | Stack: %d bytes\n",
                        taskName, coreId, priority, stackSize);
    }
}

// =============================================================================
// GLOBAL UTILITY FUNCTIONS
// =============================================================================

bool initializeSystem(const SystemConfig_t& config) {
    return systemInitializer.initializeSystem(config);
}

void shutdownSystem() {
    systemInitializer.shutdownSystem();
}

bool isSystemHealthy() {
    return systemInitializer.isSystemHealthy();
}

void printSystemStatus() {
    systemInitializer.printSystemStatus();
}

MultiCoreCar* getMultiCoreCar() {
    return systemInitializer.getMultiCoreCar();
}

ROSCommTask* getROSCommTask() {
    return systemInitializer.getROSCommTask();
}

InterCoreCommunication* getInterCoreCommunication() {
    return systemInitializer.getInterCoreCommunication();
}
