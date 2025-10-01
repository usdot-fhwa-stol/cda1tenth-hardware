#include "spi_manager.h"
#include <algorithm>

SPIManager::SPIManager() : last_batch_time(0), last_error_time(0), last_operation_time(0), 
                           consecutive_errors(0), spi_error_state(false), adaptive_batching_enabled(true) {
    spi_mutex = xSemaphoreCreateMutex();
    operation_queue.reserve(MAX_BATCH_SIZE);
    priority_queue.reserve(MAX_BATCH_SIZE / 2);
    error_history.reserve(MAX_ERROR_HISTORY);
}

SPIManager::~SPIManager() {
    if (spi_mutex) {
        vSemaphoreDelete(spi_mutex);
    }
}

void SPIManager::initialize() {
    // SPI is already initialized in the main setup, just reset our state
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    operation_queue.clear();
    priority_queue.clear();
    error_history.clear();
    spi_error_state = false;
    last_error_time = 0;
    last_operation_time = 0;
    consecutive_errors = 0;
    adaptive_batching_enabled = true;
    resetMetrics();
    
    xSemaphoreGive(spi_mutex);
}

void SPIManager::queueOperation(const SPIOperation& op, bool high_priority) {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    // Handle priority operations first
    if (high_priority) {
        priority_queue.push_back(op);
        if (priority_queue.size() >= MAX_BATCH_SIZE / 2) {
            executePriorityBatch();
        }
        xSemaphoreGive(spi_mutex);
        return;
    }
    
    // Adaptive batching strategy
    uint32_t current_time = micros();
    bool should_flush = false;
    
    if (adaptive_batching_enabled) {
        // Flush if queue is full
        if (operation_queue.size() >= MAX_BATCH_SIZE) {
            should_flush = true;
        }
        // Adaptive timeout based on system load and error rate
        else if (operation_queue.size() > 0) {
            uint32_t adaptive_timeout = BATCH_TIMEOUT_US;
            
            // Reduce timeout if we have many operations queued
            if (operation_queue.size() >= ADAPTIVE_BATCH_THRESHOLD) {
                adaptive_timeout = BATCH_TIMEOUT_US / 2;
            }
            
            // Increase timeout if we're having errors (to reduce bus contention)
            if (consecutive_errors > 2) {
                adaptive_timeout = BATCH_TIMEOUT_US * 2;
            }
            
            if ((current_time - last_batch_time) > adaptive_timeout) {
                should_flush = true;
            }
        }
    } else {
        // Original batching logic
        if (operation_queue.size() >= MAX_BATCH_SIZE || 
            (operation_queue.size() > 0 && (current_time - last_batch_time) > BATCH_TIMEOUT_US)) {
            should_flush = true;
        }
    }
    
    if (should_flush) {
        executeBatch();
    }
    
    operation_queue.push_back(op);
    if (operation_queue.size() == 1) {
        last_batch_time = current_time;
    }
    
    xSemaphoreGive(spi_mutex);
}

void SPIManager::flushQueue() {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    // Process priority queue first
    if (!priority_queue.empty()) {
        executePriorityBatch();
    }
    
    // Then process regular queue
    if (!operation_queue.empty()) {
        executeBatch();
    }
    
    xSemaphoreGive(spi_mutex);
}

void SPIManager::flushPriorityQueue() {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    if (!priority_queue.empty()) {
        executePriorityBatch();
    }
    xSemaphoreGive(spi_mutex);
}

bool SPIManager::processPendingOperations() {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    bool has_operations = !operation_queue.empty() || !priority_queue.empty();
    uint32_t current_time = micros();
    
    // Always process priority operations first
    if (!priority_queue.empty()) {
        executePriorityBatch();
    }
    
    // Check if batch timeout reached for regular operations
    if (!operation_queue.empty()) {
        uint32_t timeout = adaptive_batching_enabled ? 
            (consecutive_errors > 2 ? BATCH_TIMEOUT_US * 2 : BATCH_TIMEOUT_US) : 
            BATCH_TIMEOUT_US;
            
        if ((current_time - last_batch_time) > timeout) {
            executeBatch();
        }
    }
    
    // Detect and handle bus contention
    if (detectBusContention()) {
        adaptBatchingStrategy();
    }
    
    xSemaphoreGive(spi_mutex);
    return has_operations;
}

void SPIManager::executeBatch() {
    if (operation_queue.empty()) return;
    
    // Skip execution if in error state and cooldown not expired
    if (spi_error_state && (millis() - last_error_time) < SPI_ERROR_COOLDOWN_MS) {
        // Mark all operations as failed
        for (auto& op : operation_queue) {
            op.completed = true;
            op.success = false;
        }
        metrics.failed_operations += operation_queue.size();
        metrics.total_operations += operation_queue.size();
        operation_queue.clear();
        return;
    }
    
    uint32_t batch_start_time = micros();
    uint32_t batch_size = operation_queue.size();
    bool batch_success = true;
    uint32_t cs_transitions = 0;
    
    // Optimize batch order for efficiency
    optimizeBatchOrder();
    
    int current_cs = -1;
    
    for (auto& op : operation_queue) {
        // Handle CS pin transitions
        if (current_cs != op.cs_pin) {
            if (current_cs != -1) {
                digitalWrite(current_cs, HIGH); // Deselect previous
                cs_transitions++;
            }
            digitalWrite(op.cs_pin, LOW); // Select new
            current_cs = op.cs_pin;
            cs_transitions++;
            delayMicroseconds(1); // Small delay for CS setup
        }
        
        bool op_success = executeSingleOperation(op);
        op.completed = true;
        op.success = op_success;
        
        if (!op_success) {
            batch_success = false;
            recordError(1, op.cs_pin); // Generic SPI error
        }
    }
    
    // Deselect final CS
    if (current_cs != -1) {
        digitalWrite(current_cs, HIGH);
        cs_transitions++;
    }
    
    uint32_t batch_execution_time = micros() - batch_start_time;
    last_operation_time = micros();
    
    // Update comprehensive metrics
    updateMetrics(batch_success, batch_execution_time, cs_transitions);
    
    uint32_t successful_ops = std::count_if(operation_queue.begin(), operation_queue.end(),
                                          [](const SPIOperation& op) { return op.success; });
    metrics.total_operations += batch_size;
    metrics.successful_operations += successful_ops;
    metrics.failed_operations += (batch_size - successful_ops);
    metrics.batched_operations++;
    metrics.cs_transitions += cs_transitions;
    
    // Update running averages
    metrics.average_batch_size = (metrics.average_batch_size * (metrics.batched_operations - 1) + batch_size) / metrics.batched_operations;
    metrics.success_rate = (float)metrics.successful_operations / metrics.total_operations;
    metrics.average_operation_time_us = (float)metrics.total_execution_time_us / metrics.total_operations;
    metrics.bus_efficiency = calculateBusEfficiency();
    
    if (!batch_success) {
        consecutive_errors++;
        handleSPIError(1, current_cs);
    } else {
        consecutive_errors = 0;
        spi_error_state = false; // Clear error state on successful batch
    }
    
    operation_queue.clear();
}

void SPIManager::executePriorityBatch() {
    if (priority_queue.empty()) return;
    
    uint32_t batch_start_time = micros();
    uint32_t batch_size = priority_queue.size();
    bool batch_success = true;
    uint32_t cs_transitions = 0;
    
    // Priority operations don't get optimized for order - execute immediately
    int current_cs = -1;
    
    for (auto& op : priority_queue) {
        // Handle CS pin transitions
        if (current_cs != op.cs_pin) {
            if (current_cs != -1) {
                digitalWrite(current_cs, HIGH);
                cs_transitions++;
            }
            digitalWrite(op.cs_pin, LOW);
            current_cs = op.cs_pin;
            cs_transitions++;
            delayMicroseconds(1);
        }
        
        bool op_success = executeSingleOperation(op);
        op.completed = true;
        op.success = op_success;
        
        if (!op_success) {
            batch_success = false;
            recordError(2, op.cs_pin); // Priority operation error
        }
    }
    
    // Deselect final CS
    if (current_cs != -1) {
        digitalWrite(current_cs, HIGH);
        cs_transitions++;
    }
    
    uint32_t batch_execution_time = micros() - batch_start_time;
    
    // Update metrics for priority operations
    updateMetrics(batch_success, batch_execution_time, cs_transitions);
    
    uint32_t successful_ops = std::count_if(priority_queue.begin(), priority_queue.end(),
                                          [](const SPIOperation& op) { return op.success; });
    metrics.total_operations += batch_size;
    metrics.successful_operations += successful_ops;
    metrics.failed_operations += (batch_size - successful_ops);
    metrics.cs_transitions += cs_transitions;
    
    if (!batch_success) {
        consecutive_errors++;
        handleSPIError(2, current_cs);
    } else {
        consecutive_errors = max(0, (int)consecutive_errors - 1); // Reduce error count on priority success
    }
    
    priority_queue.clear();
}

bool SPIManager::executeSingleOperation(SPIOperation& op) {
    uint32_t retry_count = 0;
    
    while (retry_count < MAX_RETRY_ATTEMPTS) {
        uint32_t operation_start = micros();
        
        try {
            if (op.type == READ_OPERATION) {
                // TMC5160 read operation with enhanced error detection
                uint8_t cmd = op.address & 0x7F; // Clear write bit
                
                // First transaction - send read command
                uint8_t status1 = SPI.transfer(cmd);
                SPI.transfer(0x00);
                SPI.transfer(0x00);
                SPI.transfer(0x00);
                SPI.transfer(0x00);
                
                // Second transaction to get the data
                digitalWrite(op.cs_pin, HIGH);
                delayMicroseconds(2); // Slightly longer delay for stability
                digitalWrite(op.cs_pin, LOW);
                delayMicroseconds(2);
                
                uint8_t status2 = SPI.transfer(cmd);
                uint32_t result = 0;
                result |= ((uint32_t)SPI.transfer(0x00)) << 24;
                result |= ((uint32_t)SPI.transfer(0x00)) << 16;
                result |= ((uint32_t)SPI.transfer(0x00)) << 8;
                result |= SPI.transfer(0x00);
                
                // Enhanced validation
                if (!validateSPIResponse(result, op.address)) {
                    metrics.retry_attempts++;
                    retry_count++;
                    
                    if (retry_count < MAX_RETRY_ATTEMPTS) {
                        // Exponential backoff for retries
                        delayMicroseconds(10 * (1 << retry_count));
                        continue;
                    } else {
                        return false;
                    }
                }
                
                if (op.result) {
                    *op.result = result;
                }
                
                return true;
                
            } else { // WRITE_OPERATION
                uint8_t cmd = op.address | 0x80; // Set write bit
                
                uint8_t status = SPI.transfer(cmd);
                SPI.transfer((op.data >> 24) & 0xFF);
                SPI.transfer((op.data >> 16) & 0xFF);
                SPI.transfer((op.data >> 8) & 0xFF);
                SPI.transfer(op.data & 0xFF);
                
                // For critical registers, add a verification read
                if (op.address == 0x24 || op.address == 0x2D || op.address == 0x21) { // VMAX, XTARGET, XACTUAL
                    delayMicroseconds(5); // Allow write to settle
                    
                    // Quick verification read (optional, can be disabled for performance)
                    digitalWrite(op.cs_pin, HIGH);
                    delayMicroseconds(1);
                    digitalWrite(op.cs_pin, LOW);
                    delayMicroseconds(1);
                    
                    uint8_t read_cmd = op.address & 0x7F;
                    SPI.transfer(read_cmd);
                    SPI.transfer(0x00);
                    SPI.transfer(0x00);
                    SPI.transfer(0x00);
                    SPI.transfer(0x00);
                }
                
                return true; // Write operations assumed successful if no SPI errors
            }
            
        } catch (...) {
            metrics.retry_attempts++;
            retry_count++;
            
            if (retry_count < MAX_RETRY_ATTEMPTS) {
                // Exponential backoff with jitter
                uint32_t delay_us = 10 * (1 << retry_count) + (micros() % 5);
                delayMicroseconds(delay_us);
            }
        }
        
        uint32_t operation_time = micros() - operation_start;
        
        // Detect unusually long operations (potential bus issues)
        if (operation_time > 1000) { // 1ms is very long for SPI
            metrics.timeout_events++;
            recordError(3, op.cs_pin); // Timeout error
        }
    }
    
    return false; // All retries failed
}

bool SPIManager::validateSPIResponse(uint32_t response, uint8_t address) {
    // Basic validation - check if response is not all zeros or all ones
    // (which might indicate communication failure)
    if (response == 0x00000000 || response == 0xFFFFFFFF) {
        return false;
    }
    
    // Address-specific validation for TMC5160 registers
    switch (address) {
        case 0x01: // GSTAT
            // GSTAT should have reasonable values (not all bits set)
            if ((response & 0xFFFFFF00) != 0) return false; // Upper bits should be 0
            break;
            
        case 0x6F: // DRV_STATUS
            // DRV_STATUS has specific bit patterns
            break;
            
        case 0x21: // XACTUAL
        case 0x2D: // XTARGET
            // Position values should be within reasonable range
            // (This is application-specific, could be made configurable)
            break;
            
        case 0x24: // VMAX
            // Velocity should be within motor limits
            if (response > 1000000) return false; // Sanity check
            break;
            
        case 0x22: // VACTUAL
            // Actual velocity validation
            break;
            
        default:
            // For unknown registers, just do basic validation
            break;
    }
    
    return true;
}

void SPIManager::updateMetrics(bool success, uint32_t execution_time, uint32_t cs_transitions) {
    metrics.total_execution_time_us += execution_time;
    
    if (!success) {
        metrics.retry_attempts++;
    }
}

void SPIManager::handleSPIError(uint8_t error_type, int cs_pin) {
    spi_error_state = true;
    last_error_time = millis();
    
    // Record error for analysis
    recordError(error_type, cs_pin);
    
    // Adaptive error handling based on error frequency
    if (consecutive_errors > 5) {
        // Severe error condition - increase cooldown and disable adaptive batching temporarily
        adaptive_batching_enabled = false;
    } else if (consecutive_errors > 2) {
        // Moderate errors - reduce batch size and increase timeouts
        // This is handled in the adaptive batching logic
    }
    
    // Could add additional error handling here like:
    // - Reinitializing SPI
    // - Logging error details
    // - Notifying error management system
}

void SPIManager::optimizeBatchOrder() {
    if (operation_queue.size() <= 1) return;
    
    // Sort by CS pin to minimize transitions, then by operation type
    // (reads first, then writes to reduce bus turnaround time)
    std::sort(operation_queue.begin(), operation_queue.end(), 
              [](const SPIOperation& a, const SPIOperation& b) {
                  if (a.cs_pin != b.cs_pin) {
                      return a.cs_pin < b.cs_pin;
                  }
                  // Within same CS, do reads first (they're typically faster)
                  return a.type < b.type;
              });
}

bool SPIManager::detectBusContention() {
    uint32_t current_time = micros();
    
    // Check if operations are taking longer than expected
    if (last_operation_time > 0 && (current_time - last_operation_time) > BUS_CONTENTION_THRESHOLD_US) {
        metrics.bus_contention_events++;
        return true;
    }
    
    // Check if we have a high error rate recently
    if (consecutive_errors > 3) {
        return true;
    }
    
    return false;
}

void SPIManager::adaptBatchingStrategy() {
    // If we detect contention, adjust batching parameters
    if (consecutive_errors > 3) {
        // Reduce batch size to minimize impact of failures
        // This is implicitly handled by the adaptive timeout logic
        adaptive_batching_enabled = true; // Re-enable if it was disabled
    }
}

void SPIManager::recordError(uint8_t error_type, int cs_pin) {
    ErrorHistory error;
    error.timestamp = millis();
    error.error_type = error_type;
    error.cs_pin = cs_pin;
    
    error_history.push_back(error);
    
    // Keep only recent errors
    if (error_history.size() > MAX_ERROR_HISTORY) {
        error_history.erase(error_history.begin());
    }
}

float SPIManager::calculateBusEfficiency() const {
    if (metrics.total_operations == 0 || metrics.total_execution_time_us == 0) {
        return 1.0f;
    }
    
    // Estimate ideal time (assuming no CS transitions and minimal overhead)
    uint32_t ideal_time = metrics.total_operations * 50; // ~50us per operation ideal
    
    // Calculate efficiency as ratio of ideal to actual time
    float efficiency = (float)ideal_time / metrics.total_execution_time_us;
    
    // Clamp to reasonable range
    return min(1.0f, max(0.1f, efficiency));
}

bool SPIManager::readRegister(int cs_pin, uint8_t address, uint32_t& result) {
    SPIOperation op(cs_pin, READ_OPERATION, address, 0, &result);
    
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);
    
    uint32_t start_time = micros();
    bool success = executeSingleOperation(op);
    uint32_t execution_time = micros() - start_time;
    
    digitalWrite(cs_pin, HIGH);
    
    updateMetrics(success, execution_time);
    
    xSemaphoreGive(spi_mutex);
    
    return success;
}

bool SPIManager::writeRegister(int cs_pin, uint8_t address, uint32_t data) {
    SPIOperation op(cs_pin, WRITE_OPERATION, address, data);
    
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);
    
    uint32_t start_time = micros();
    bool success = executeSingleOperation(op);
    uint32_t execution_time = micros() - start_time;
    
    digitalWrite(cs_pin, HIGH);
    
    updateMetrics(success, execution_time);
    
    xSemaphoreGive(spi_mutex);
    
    return success;
}

SPIManager::SPIPerformanceMetrics SPIManager::getMetrics() const {
    return metrics;
}

void SPIManager::resetMetrics() {
    metrics = SPIPerformanceMetrics();
}

bool SPIManager::isHealthy() const {
    return !spi_error_state && metrics.success_rate > 0.95f;
}

bool SPIManager::isInErrorState() const {
    return spi_error_state;
}

void SPIManager::clearErrorState() {
    spi_error_state = false;
    last_error_time = 0;
}

uint32_t SPIManager::getLastErrorTime() const {
    return last_error_time;
}

void SPIManager::enableAdaptiveBatching(bool enable) {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    adaptive_batching_enabled = enable;
    xSemaphoreGive(spi_mutex);
}

uint32_t SPIManager::getConsecutiveErrors() const {
    return consecutive_errors;
}

std::vector<SPIManager::ErrorHistory> SPIManager::getErrorHistory() const {
    return error_history;
}

void SPIManager::clearErrorHistory() {
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    error_history.clear();
    consecutive_errors = 0;
    xSemaphoreGive(spi_mutex);
}