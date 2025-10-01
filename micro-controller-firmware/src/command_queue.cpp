#include "command_queue.h"

CommandQueue::CommandQueue() {
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        // Handle error - could log via debug system when available
    }
}

CommandQueue::~CommandQueue() {
    if (mutex != NULL) {
        vSemaphoreDelete(mutex);
    }
}

bool CommandQueue::enqueue(const ControlCommand& cmd) {
    if (mutex == NULL) return false;
    
    // Take mutex with timeout to avoid blocking indefinitely
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    
    size_t current_head = head.load();
    size_t current_tail = tail.load();
    size_t next_head = (current_head + 1) % QUEUE_SIZE;
    
    // Check if queue is full
    if (next_head == current_tail) {
        // Queue is full - for critical commands (STOP), remove oldest non-critical command
        if (cmd.type == CommandType::STOP) {
            // Find and remove oldest non-STOP command to make room
            for (size_t i = current_tail; i != current_head; i = (i + 1) % QUEUE_SIZE) {
                if (buffer[i].type != CommandType::STOP) {
                    // Remove this command by shifting others
                    for (size_t j = i; j != current_head; j = (j + 1) % QUEUE_SIZE) {
                        size_t next_j = (j + 1) % QUEUE_SIZE;
                        if (next_j != current_head) {
                            buffer[j] = buffer[next_j];
                        }
                    }
                    head.store((current_head - 1 + QUEUE_SIZE) % QUEUE_SIZE);
                    current_head = head.load();
                    next_head = (current_head + 1) % QUEUE_SIZE;
                    break;
                }
            }
            
            // If still full (all commands are STOP), just overwrite oldest
            if (next_head == current_tail) {
                tail.store((current_tail + 1) % QUEUE_SIZE);
            }
        } else {
            xSemaphoreGive(mutex);
            return false;  // Queue full, non-critical command rejected
        }
    }
    
    // Insert command with priority handling
    // For STOP commands, insert at front for immediate processing
    if (cmd.type == CommandType::STOP) {
        // Insert at tail position (front of queue)
        size_t insert_pos = (current_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        buffer[insert_pos] = cmd;
        tail.store(insert_pos);
    } else {
        // Insert at head position (back of queue) for normal priority
        buffer[current_head] = cmd;
        head.store(next_head);
    }
    
    xSemaphoreGive(mutex);
    return true;
}

bool CommandQueue::dequeue(ControlCommand& cmd) {
    if (mutex == NULL) return false;
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;
    }
    
    size_t current_head = head.load();
    size_t current_tail = tail.load();
    
    // Check if queue is empty
    if (current_head == current_tail) {
        xSemaphoreGive(mutex);
        return false;
    }
    
    // Get command from tail (front of queue)
    cmd = buffer[current_tail];
    
    // Check if command is stale
    if (isCommandStale(cmd)) {
        // Remove stale command and try next
        tail.store((current_tail + 1) % QUEUE_SIZE);
        xSemaphoreGive(mutex);
        return dequeue(cmd);  // Recursive call to get next command
    }
    
    tail.store((current_tail + 1) % QUEUE_SIZE);
    
    xSemaphoreGive(mutex);
    return true;
}

void CommandQueue::clear() {
    if (mutex == NULL) return;
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        head.store(0);
        tail.store(0);
        xSemaphoreGive(mutex);
    }
}

size_t CommandQueue::size() const {
    size_t current_head = head.load();
    size_t current_tail = tail.load();
    
    if (current_head >= current_tail) {
        return current_head - current_tail;
    } else {
        return QUEUE_SIZE - current_tail + current_head;
    }
}

bool CommandQueue::isEmpty() const {
    return head.load() == tail.load();
}

bool CommandQueue::isFull() const {
    size_t current_head = head.load();
    size_t current_tail = tail.load();
    return ((current_head + 1) % QUEUE_SIZE) == current_tail;
}

bool CommandQueue::peek(ControlCommand& cmd) const {
    if (mutex == NULL) return false;
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;
    }
    
    size_t current_head = head.load();
    size_t current_tail = tail.load();
    
    // Check if queue is empty
    if (current_head == current_tail) {
        xSemaphoreGive(mutex);
        return false;
    }
    
    cmd = buffer[current_tail];
    
    xSemaphoreGive(mutex);
    return true;
}

void CommandQueue::cleanupStaleCommands() {
    if (mutex == NULL) return;
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    size_t current_head = head.load();
    size_t current_tail = tail.load();
    
    // Remove stale commands from front of queue
    while (current_tail != current_head) {
        if (isCommandStale(buffer[current_tail])) {
            current_tail = (current_tail + 1) % QUEUE_SIZE;
        } else {
            break;  // First non-stale command found
        }
    }
    
    tail.store(current_tail);
    
    xSemaphoreGive(mutex);
}