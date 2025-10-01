#ifndef COMMAND_QUEUE_H
#define COMMAND_QUEUE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>

// Command types with priority levels
enum class CommandType : uint8_t {
    STOP = 0,      // Highest priority
    SPEED = 1,     // Medium priority  
    STEERING = 2   // Lower priority
};

struct ControlCommand {
    CommandType type;
    float value;
    uint32_t timestamp;
    
    ControlCommand() : type(CommandType::STOP), value(0.0f), timestamp(0) {}
    ControlCommand(CommandType t, float v) : type(t), value(v), timestamp(millis()) {}
};

class CommandQueue {
private:
    static const size_t QUEUE_SIZE = 32;  // Power of 2 for efficient modulo
    static const uint32_t COMMAND_TIMEOUT_MS = 1000;  // Commands expire after 1 second
    
    ControlCommand buffer[QUEUE_SIZE];
    std::atomic<size_t> head{0};
    std::atomic<size_t> tail{0};
    SemaphoreHandle_t mutex;
    
    // Helper function to get priority value (lower number = higher priority)
    uint8_t getPriority(CommandType type) const {
        return static_cast<uint8_t>(type);
    }
    
    // Check if command is stale
    bool isCommandStale(const ControlCommand& cmd) const {
        return (millis() - cmd.timestamp) > COMMAND_TIMEOUT_MS;
    }

public:
    CommandQueue();
    ~CommandQueue();
    
    // Thread-safe operations
    bool enqueue(const ControlCommand& cmd);
    bool dequeue(ControlCommand& cmd);
    void clear();
    size_t size() const;
    bool isEmpty() const;
    bool isFull() const;
    
    // Get highest priority command without removing it
    bool peek(ControlCommand& cmd) const;
    
    // Remove stale commands
    void cleanupStaleCommands();
};

#endif // COMMAND_QUEUE_H