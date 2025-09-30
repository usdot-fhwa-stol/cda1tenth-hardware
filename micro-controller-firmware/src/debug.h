#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

// Debug counters (extern declarations)
extern uint32_t twist_callback_count;
extern uint32_t odom_publish_count;

// Debug logging function
void logDebug(const char* message);

#endif // DEBUG_H
