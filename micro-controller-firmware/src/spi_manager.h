#ifndef SPI_MANAGER_H
#define SPI_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>

class SPIManager {
public:
    enum OperationType {
        READ_OPERATION,
        WRITE_OPERATION
    };
    
    struct SPIOperation {
        int cs_pin;
        OperationType type;
        uint8_t address;
        uint32_t data;
        uint32_t* result;
        bool completed;
        bool success;
        
        SPIOperation(int cs, OperationType op_type, uint8_t addr, uint32_t data_val = 0, uint32_t* res = nullptr)
            : cs_pin(cs), type(op_type), address(addr), data(data_val), result(res), completed(false), success(false) {}
    };
    
    struct SPIPerformanceMetrics {
        uint32_t total_operations;
        uint32_t successful_operations;
        uint32_t failed_operations;
        uint32_t batched_operations;
        uint32_t average_batch_size;
        uint32_t total_execution_time_us;
        uint32_t bus_contention_events;
        uint32_t retry_attempts;
        uint32_t timeout_events;
        uint32_t cs_transitions;
        float success_rate;
        float average_operation_time_us;
        float bus_efficiency; // Ratio of useful operations to total bus time
        
        SPIPerformanceMetrics() : total_operations(0), successful_operations(0), failed_operations(0),
                                 batched_operations(0), average_batch_size(0), total_execution_time_us(0),
                                 bus_contention_events(0), retry_attempts(0), timeout_events(0), cs_transitions(0),
                                 success_rate(1.0f), average_operation_time_us(0.0f), bus_efficiency(1.0f) {}
    };

private:
    static const uint32_t MAX_BATCH_SIZE = 16;
    static const uint32_t BATCH_TIMEOUT_US = 1000; // 1ms timeout for batching
    static const uint32_t MAX_RETRY_ATTEMPTS = 3;
    static const uint32_t SPI_ERROR_COOLDOWN_MS = 100;
    static const uint32_t ADAPTIVE_BATCH_THRESHOLD = 8; // Threshold for adaptive batching
    static const uint32_t BUS_CONTENTION_THRESHOLD_US = 5000; // 5ms threshold for contention detection
    
    std::vector<SPIOperation> operation_queue;
    std::vector<SPIOperation> priority_queue; // High priority operations (e.g., emergency stops)
    SemaphoreHandle_t spi_mutex;
    SPIPerformanceMetrics metrics;
    
    uint32_t last_batch_time;
    uint32_t last_error_time;
    uint32_t last_operation_time;
    uint32_t consecutive_errors;
    bool spi_error_state;
    bool adaptive_batching_enabled;
    
    // Enhanced error tracking
    struct ErrorHistory {
        uint32_t timestamp;
        uint8_t error_type;
        int cs_pin;
    };
    std::vector<ErrorHistory> error_history;
    static const uint32_t MAX_ERROR_HISTORY = 10;
    
    // Helper functions
    bool executeSingleOperation(SPIOperation& op);
    void executeBatch();
    void executePriorityBatch();
    bool validateSPIResponse(uint32_t response, uint8_t address);
    void updateMetrics(bool success, uint32_t execution_time, uint32_t cs_transitions = 0);
    void handleSPIError(uint8_t error_type = 0, int cs_pin = -1);
    void optimizeBatchOrder();
    bool detectBusContention();
    void adaptBatchingStrategy();
    void recordError(uint8_t error_type, int cs_pin);
    float calculateBusEfficiency() const;

public:
    SPIManager();
    ~SPIManager();
    
    void initialize();
    
    // Batched operations
    void queueOperation(const SPIOperation& op, bool high_priority = false);
    void flushQueue();
    void flushPriorityQueue();
    bool processPendingOperations();
    void enableAdaptiveBatching(bool enable = true);
    
    // Direct operations with error handling
    bool readRegister(int cs_pin, uint8_t address, uint32_t& result);
    bool writeRegister(int cs_pin, uint8_t address, uint32_t data);
    
    // Performance monitoring
    SPIPerformanceMetrics getMetrics() const;
    void resetMetrics();
    bool isHealthy() const;
    
    // Error handling
    bool isInErrorState() const;
    void clearErrorState();
    uint32_t getLastErrorTime() const;
    uint32_t getConsecutiveErrors() const;
    std::vector<ErrorHistory> getErrorHistory() const;
    void clearErrorHistory();
};

#endif // SPI_MANAGER_H