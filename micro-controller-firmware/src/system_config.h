#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// COMPILE-TIME CONFIGURATION CONSTANTS
// =============================================================================

// Hardware pin definitions
#define NEOPIXEL_PIN 48  // WS2812 NeoPixel RGB LED pin
#define NEOPIXEL_COUNT 1  // Number of NeoPixels

// Task priorities (higher number = higher priority)
#define MOTOR_CONTROL_TASK_PRIORITY     24
#define ROS_COMM_TASK_PRIORITY          10

// Task configuration
#define MOTOR_TASK_STACK_SIZE           4096    // Stack size for motor control task
#define MOTOR_TASK_CORE_ID              0       // Core ID for motor control task (0 or 1)
#define ROS_TASK_CORE_ID                1       // Core ID for ROS communication task (0 or 1)

// Task timing parameters
#define MOTOR_CONTROL_FREQUENCY_HZ      1000    // 1kHz motor control loop
#define MOTOR_CONTROL_PERIOD_MS         (1000 / MOTOR_CONTROL_FREQUENCY_HZ)
#define ROS_SPIN_TIMEOUT_MS             10      // ROS executor timeout

// Task stack sizes (in words, not bytes)
#define MOTOR_CONTROL_TASK_STACK_SIZE   4096
#define ROS_COMM_TASK_STACK_SIZE        8192

// Queue configuration
#define COMMAND_QUEUE_SIZE              8
#define STATUS_QUEUE_SIZE               8

// Communication parameters
#define SERIAL_BAUD_RATE                921600
#define USB_TX_BUFFER_SIZE              1024
#define USB_RX_BUFFER_SIZE              1024

// Safety and timeout parameters
#define COMMAND_TIMEOUT_MS              500     // Max time without new commands
#define WATCHDOG_TIMEOUT_MS             1000    // Task watchdog timeout
#define AGENT_PING_INTERVAL_MS          200     // ROS agent ping interval
#define AGENT_PING_TIMEOUT_MS           100     // ROS agent ping timeout

// Performance monitoring
#define PERFORMANCE_SAMPLE_SIZE         100     // Number of samples for averaging

// ROS Communication Task Configuration
#define ROS_TASK_FREQUENCY_HZ           100     // ROS task execution frequency
#define ROS_TASK_PERIOD_MS              (1000 / ROS_TASK_FREQUENCY_HZ)
#define ROS_MOTOR_RPM_PUBLISH_RATE_HZ   10      // Motor RPM publishing rate
#define ROS_MOTOR_RPM_PUBLISH_PERIOD_MS (1000 / ROS_MOTOR_RPM_PUBLISH_RATE_HZ)
#define ROS_ODOMETRY_LOG_RATE_HZ        4       // Odometry logging rate
#define ROS_ODOMETRY_LOG_PERIOD_MS      (1000 / ROS_ODOMETRY_LOG_RATE_HZ)
#define ROS_GEOMETRY_LOG_RATE_HZ        1       // Geometry logging rate
#define ROS_GEOMETRY_LOG_PERIOD_MS      (1000 / ROS_GEOMETRY_LOG_RATE_HZ)

// ROS Message Validation Limits
#define ROS_TWIST_LINEAR_X_MAX          10.0f   // Maximum linear velocity X (m/s)
#define ROS_TWIST_LINEAR_Y_MAX          1.0f    // Maximum linear velocity Y (m/s)
#define ROS_TWIST_LINEAR_Z_MAX          1.0f    // Maximum linear velocity Z (m/s)
#define ROS_TWIST_ANGULAR_X_MAX         10.0f   // Maximum angular velocity X (rad/s)
#define ROS_TWIST_ANGULAR_Y_MAX         10.0f   // Maximum angular velocity Y (rad/s)
#define ROS_TWIST_ANGULAR_Z_MAX         10.0f   // Maximum angular velocity Z (rad/s)

// Vehicle Geometry Constants
#define DEFAULT_WHEELBASE               0.185f  // Default wheelbase in meters
#define DEFAULT_TRACK_WIDTH             0.15f   // Default track width in meters
#define MAX_STEERING_ANGLE_DEG          45.0f   // Maximum steering angle in degrees
#define MAX_STEERING_ANGLE_RAD          (MAX_STEERING_ANGLE_DEG * M_PI / 180.0f)

// ROS Performance Monitoring
#define ROS_ERROR_REPORT_INTERVAL_MS    1000    // Error reporting interval
#define ROS_MAX_MISSED_DEADLINES        10      // Maximum missed deadlines before warning

// Motor Command Validation Limits
#define MAX_STEERING_ANGLE_COMMAND      45.0f   // Maximum steering angle for commands (degrees)
#define MAX_DRIVE_SPEED                 1000.0f // Maximum drive speed (RPM or m/s)
#define MAX_WHEELBASE                   1.0f    // Maximum wheelbase (meters)
#define MAX_TRACK_WIDTH                 1.0f    // Maximum track width (meters)

// Motor Status Validation Limits
#define MAX_MOTOR_RPM                   2000.0f // Maximum motor RPM
#define MAX_STEERING_ANGLE_STATUS       45.0f   // Maximum steering angle for status (degrees)

// Queue Configuration
#define MAX_QUEUE_SIZE_FOR_UTILIZATION  10      // Maximum queue size for utilization calculation
#define MUTEX_TIMEOUT_MS                10      // Mutex timeout in milliseconds
#define MAX_ACCEPTABLE_ERRORS           100     // Maximum acceptable errors before warning

// =============================================================================
// SYSTEM CONFIGURATION STRUCTURE
// =============================================================================

typedef struct {
    // Task priorities
    uint8_t motor_task_priority;
    uint8_t ros_task_priority;
    
    // Task configuration
    uint32_t motor_task_stack_size;
    uint8_t motor_task_core_id;
    uint8_t ros_task_core_id;
    
    // Timing parameters
    uint32_t motor_control_frequency_hz;
    uint32_t ros_spin_timeout_ms;
    
    // Queue sizes
    uint8_t command_queue_size;
    uint8_t status_queue_size;
    
    // Communication parameters
    uint32_t serial_baud_rate;
    uint16_t usb_tx_buffer_size;
    uint16_t usb_rx_buffer_size;
    
    // Safety parameters
    uint32_t command_timeout_ms;
    uint32_t watchdog_timeout_ms;
    uint32_t agent_ping_interval_ms;
    uint32_t agent_ping_timeout_ms;
    
    // Performance monitoring
    uint16_t performance_sample_size;
    
    // ROS Communication Task Configuration
    uint32_t ros_task_frequency_hz;
    uint32_t ros_motor_rpm_publish_rate_hz;
    uint32_t ros_odometry_log_rate_hz;
    uint32_t ros_geometry_log_rate_hz;
    
    // ROS Message Validation Limits
    float ros_twist_linear_x_max;
    float ros_twist_linear_y_max;
    float ros_twist_linear_z_max;
    float ros_twist_angular_x_max;
    float ros_twist_angular_y_max;
    float ros_twist_angular_z_max;
    
    // Vehicle Geometry Constants
    float default_wheelbase;
    float default_track_width;
    float max_steering_angle_deg;
    
    // ROS Performance Monitoring
    uint32_t ros_error_report_interval_ms;
    uint32_t ros_max_missed_deadlines;
    
    // Motor Command Validation Limits
    float max_steering_angle_command;
    float max_drive_speed;
    float max_wheelbase;
    float max_track_width;
    
    // Motor Status Validation Limits
    float max_motor_rpm;
    float max_steering_angle_status;
    
    // Queue Configuration
    uint8_t max_queue_size_for_utilization;
    uint32_t mutex_timeout_ms;
    uint32_t max_acceptable_errors;
} SystemConfig_t;

// =============================================================================
// INTER-CORE COMMUNICATION STRUCTURES
// =============================================================================

// Command structure sent from ROS core to Motor Control core
typedef struct {
    float steering_angle;           // Desired steering angle in degrees
    float drive_speed;              // Desired drive speed in RPM
    float wheelbase;                // Vehicle wheelbase in meters
    float track_width;              // Vehicle track width in meters
    uint32_t timestamp;             // Timestamp when command was created (microseconds)
    bool emergency_stop;            // Emergency stop flag
    uint8_t sequence_id;            // Sequence ID for command tracking
} MotorCommand_t;

// Status structure sent from Motor Control core to ROS core
typedef struct {
    float right_motor_rpm;          // Current right motor RPM
    float left_motor_rpm;           // Current left motor RPM
    float steering_angle;           // Current steering angle in degrees
    float steering_position;        // Raw steering position sensor value
    uint32_t timestamp;             // Timestamp when status was created (microseconds)
    uint8_t error_flags;            // Error flags bitfield
    uint8_t sequence_id;            // Sequence ID for status tracking
    bool motor_control_active;      // Motor control task health indicator
} MotorStatus_t;

// Error flags for MotorStatus_t.error_flags bitfield
#define ERROR_FLAG_SPI_COMM_FAILURE     (1 << 0)    // SPI communication error
#define ERROR_FLAG_MOTOR_STALL          (1 << 1)    // Motor stall detected
#define ERROR_FLAG_COMMAND_TIMEOUT      (1 << 2)    // Command timeout
#define ERROR_FLAG_QUEUE_OVERFLOW       (1 << 3)    // Queue overflow
#define ERROR_FLAG_MUTEX_TIMEOUT        (1 << 4)    // Mutex timeout
#define ERROR_FLAG_WATCHDOG_TIMEOUT     (1 << 5)    // Watchdog timeout
#define ERROR_FLAG_MEMORY_ALLOCATION    (1 << 6)    // Memory allocation failure
#define ERROR_FLAG_DRIVER_FAULT         (1 << 7)    // Motor driver fault

// =============================================================================
// PERFORMANCE MONITORING STRUCTURE
// =============================================================================

typedef struct {
    // Task execution timing (in microseconds)
    uint32_t motor_task_max_execution_us;
    uint32_t motor_task_avg_execution_us;
    uint32_t motor_task_min_execution_us;
    uint32_t ros_task_max_execution_us;
    uint32_t ros_task_avg_execution_us;
    uint32_t ros_task_min_execution_us;
    
    // Task execution counts
    uint32_t motor_task_execution_count;
    uint32_t ros_task_execution_count;
    
    // Queue statistics
    uint32_t command_queue_overruns;
    uint32_t status_queue_overruns;
    uint32_t command_queue_max_depth;
    uint32_t status_queue_max_depth;
    
    // Communication statistics
    uint32_t spi_communication_errors;
    uint32_t ros_connection_drops;
    uint32_t ros_message_send_failures;
    uint32_t ros_message_receive_count;
    
    // System health indicators
    uint32_t mutex_timeout_count;
    uint32_t watchdog_timeout_count;
    uint32_t memory_allocation_failures;
    
    // CPU utilization (percentage * 100 for integer math)
    uint16_t core0_cpu_utilization_percent_x100;
    uint16_t core1_cpu_utilization_percent_x100;
    
    // System uptime
    uint32_t system_uptime_seconds;
    
    // Last update timestamp
    uint32_t last_update_timestamp;
} PerformanceMetrics_t;

// =============================================================================
// DEFAULT CONFIGURATION
// =============================================================================

// Default system configuration
static const SystemConfig_t DEFAULT_SYSTEM_CONFIG = {
    .motor_task_priority = MOTOR_CONTROL_TASK_PRIORITY,
    .ros_task_priority = ROS_COMM_TASK_PRIORITY,
    .motor_task_stack_size = MOTOR_TASK_STACK_SIZE,
    .motor_task_core_id = MOTOR_TASK_CORE_ID,
    .ros_task_core_id = ROS_TASK_CORE_ID,
    .motor_control_frequency_hz = MOTOR_CONTROL_FREQUENCY_HZ,
    .ros_spin_timeout_ms = ROS_SPIN_TIMEOUT_MS,
    .command_queue_size = COMMAND_QUEUE_SIZE,
    .status_queue_size = STATUS_QUEUE_SIZE,
    .serial_baud_rate = SERIAL_BAUD_RATE,
    .usb_tx_buffer_size = USB_TX_BUFFER_SIZE,
    .usb_rx_buffer_size = USB_RX_BUFFER_SIZE,
    .command_timeout_ms = COMMAND_TIMEOUT_MS,
    .watchdog_timeout_ms = WATCHDOG_TIMEOUT_MS,
    .agent_ping_interval_ms = AGENT_PING_INTERVAL_MS,
    .agent_ping_timeout_ms = AGENT_PING_TIMEOUT_MS,
    .performance_sample_size = PERFORMANCE_SAMPLE_SIZE,
    
    // ROS Communication Task Configuration
    .ros_task_frequency_hz = ROS_TASK_FREQUENCY_HZ,
    .ros_motor_rpm_publish_rate_hz = ROS_MOTOR_RPM_PUBLISH_RATE_HZ,
    .ros_odometry_log_rate_hz = ROS_ODOMETRY_LOG_RATE_HZ,
    .ros_geometry_log_rate_hz = ROS_GEOMETRY_LOG_RATE_HZ,
    
    // ROS Message Validation Limits
    .ros_twist_linear_x_max = ROS_TWIST_LINEAR_X_MAX,
    .ros_twist_linear_y_max = ROS_TWIST_LINEAR_Y_MAX,
    .ros_twist_linear_z_max = ROS_TWIST_LINEAR_Z_MAX,
    .ros_twist_angular_x_max = ROS_TWIST_ANGULAR_X_MAX,
    .ros_twist_angular_y_max = ROS_TWIST_ANGULAR_Y_MAX,
    .ros_twist_angular_z_max = ROS_TWIST_ANGULAR_Z_MAX,
    
    // Vehicle Geometry Constants
    .default_wheelbase = DEFAULT_WHEELBASE,
    .default_track_width = DEFAULT_TRACK_WIDTH,
    .max_steering_angle_deg = MAX_STEERING_ANGLE_DEG,
    
    // ROS Performance Monitoring
    .ros_error_report_interval_ms = ROS_ERROR_REPORT_INTERVAL_MS,
    .ros_max_missed_deadlines = ROS_MAX_MISSED_DEADLINES,
    
    // Motor Command Validation Limits
    .max_steering_angle_command = MAX_STEERING_ANGLE_COMMAND,
    .max_drive_speed = MAX_DRIVE_SPEED,
    .max_wheelbase = MAX_WHEELBASE,
    .max_track_width = MAX_TRACK_WIDTH,
    
    // Motor Status Validation Limits
    .max_motor_rpm = MAX_MOTOR_RPM,
    .max_steering_angle_status = MAX_STEERING_ANGLE_STATUS,
    
    // Queue Configuration
    .max_queue_size_for_utilization = MAX_QUEUE_SIZE_FOR_UTILIZATION,
    .mutex_timeout_ms = MUTEX_TIMEOUT_MS,
    .max_acceptable_errors = MAX_ACCEPTABLE_ERRORS
};

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Initialize MotorCommand_t with default values
static inline void motor_command_init(MotorCommand_t* cmd) {
    if (cmd != NULL) {
        cmd->steering_angle = 0.0f;
        cmd->drive_speed = 0.0f;
        cmd->wheelbase = 0.185f;        // Default wheelbase
        cmd->track_width = 0.15f;       // Default track width
        cmd->timestamp = 0;
        cmd->emergency_stop = false;
        cmd->sequence_id = 0;
    }
}

// Initialize MotorStatus_t with default values
static inline void motor_status_init(MotorStatus_t* status) {
    if (status != NULL) {
        status->right_motor_rpm = 0.0f;
        status->left_motor_rpm = 0.0f;
        status->steering_angle = 0.0f;
        status->steering_position = 0.0f;
        status->timestamp = 0;
        status->error_flags = 0;
        status->sequence_id = 0;
        status->motor_control_active = false;
    }
}

// Initialize PerformanceMetrics_t with default values
static inline void performance_metrics_init(PerformanceMetrics_t* metrics) {
    if (metrics != NULL) {
        metrics->motor_task_max_execution_us = 0;
        metrics->motor_task_avg_execution_us = 0;
        metrics->motor_task_min_execution_us = UINT32_MAX;
        metrics->ros_task_max_execution_us = 0;
        metrics->ros_task_avg_execution_us = 0;
        metrics->ros_task_min_execution_us = UINT32_MAX;
        metrics->motor_task_execution_count = 0;
        metrics->ros_task_execution_count = 0;
        metrics->command_queue_overruns = 0;
        metrics->status_queue_overruns = 0;
        metrics->command_queue_max_depth = 0;
        metrics->status_queue_max_depth = 0;
        metrics->spi_communication_errors = 0;
        metrics->ros_connection_drops = 0;
        metrics->ros_message_send_failures = 0;
        metrics->ros_message_receive_count = 0;
        metrics->mutex_timeout_count = 0;
        metrics->watchdog_timeout_count = 0;
        metrics->memory_allocation_failures = 0;
        metrics->core0_cpu_utilization_percent_x100 = 0;
        metrics->core1_cpu_utilization_percent_x100 = 0;
        metrics->system_uptime_seconds = 0;
        metrics->last_update_timestamp = 0;
    }
}

#endif // SYSTEM_CONFIG_H