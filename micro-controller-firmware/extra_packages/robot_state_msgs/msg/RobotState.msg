# Optimized robot state message for minimal size

# Header with timestamp and frame_id
std_msgs/Header header

# IMU Data (reduced precision for size optimization)
float32 accel_x
float32 accel_y
float32 accel_z
float32 gyro_x
float32 gyro_y
float32 gyro_z

# Motor Data
float32 speed
float32 steering_angle
float32 right_motor_rpm
float32 left_motor_rpm