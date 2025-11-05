# Car configuration parameters for ESP32 micro-controller
# All values are in SI units unless otherwise specified

# Physical parameters
float64 wheelbase              # Wheelbase in meters (distance between front and rear axles)
float64 track_width            # Track width in meters (distance between left and right wheels)
float64 wheel_radius           # Wheel radius in meters

# Encoder configuration
float64 encoder_offset         # Steering encoder offset in degrees (0-360)

# Control limits
float64 max_steering_angle     # Maximum steering angle in degrees
float64 max_rpm                # Maximum motor RPM

