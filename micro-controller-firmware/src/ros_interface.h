#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msg/msg/vehicle_geometry.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

// Forward declarations
class Car;
struct SensorData;

class ROSInterface {
public:
    ROSInterface();
    ~ROSInterface();
    
    // Initialization and cleanup
    bool initialize();
    void cleanup();
    bool isConnected() const;
    
    // Main update loop
    void update();
    
    // State management
    enum ConnectionState {
        WAITING_AGENT,
        AGENT_AVAILABLE, 
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };
    
    ConnectionState getState() const;
    
    // Data publishing
    void publishIMUData(const SensorData& sensorData);
    void publishMotorData(const Car& car);
    void publishDebugData(const float* debugData, size_t dataSize);
    
    // Callback functions (static for C compatibility)
    static void twistCallback(const void* msgin);
    static void geometryCallback(const void* msgin);
    
    // Configuration
    void setVehicleGeometry(float wheelbase, float trackWidth);
    float getWheelbase() const { return wheelbase_; }
    float getTrackWidth() const { return trackWidth_; }

private:
    // ROS2 objects
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;
    rcl_allocator_t allocator_;
    
    // Publishers
    rcl_publisher_t imu_publisher_;
    rcl_publisher_t motor_data_publisher_;
    rcl_publisher_t debug_publisher_;
    
    // Subscribers
    rcl_subscription_t twist_subscriber_;
    rcl_subscription_t geometry_subscriber_;
    
    // Messages
    sensor_msgs__msg__Imu imu_msg_;
    std_msgs__msg__Float32MultiArray motor_data_msg_;
    std_msgs__msg__Float32MultiArray debug_msg_;
    geometry_msgs__msg__Twist twist_msg_;
    geometry_msg__msg__VehicleGeometry geometry_msg_;
    
    // Static data arrays to avoid malloc/free
    static float motor_data_array_[4]; // [right_rpm, left_rpm, right_current, left_current]
    static float debug_data_array_[20]; // Expanded debug data
    
    // State management
    ConnectionState current_state_;
    uint32_t last_ping_time_;
    uint32_t connection_failures_;
    uint32_t executor_timeouts_;
    
    // Vehicle geometry
    float wheelbase_;
    float trackWidth_;
    
    // Internal methods
    bool createEntities();
    void destroyEntities();
    void updateConnectionState();
    bool pingAgent();
    
    // Static callbacks need access to instance
    static ROSInterface* instance_;
};

#endif // ROS_INTERFACE_H
