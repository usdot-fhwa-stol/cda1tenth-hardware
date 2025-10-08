#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>

class CarOdometryNode : public rclcpp::Node
{
public:
    CarOdometryNode() : Node("car_odometry_node")
    {
        // Declare parameters
        this->declare_parameter("wheel_radius", 0.03);  // 3cm radius
        this->declare_parameter("wheelbase", 0.185);    // 18.5cm wheelbase
        this->declare_parameter("track_width", 0.15);  // 15cm track width
        this->declare_parameter("publish_rate", 50.0);   // 50Hz
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("child_frame_id", "base_link");
        
        // Get parameters
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        
        // Initialize odometry state
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        last_time_ = this->now();
        
        // Create publishers
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("odom_twist", 10);
        
        // Create subscribers
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&CarOdometryNode::imu_callback, this, std::placeholders::_1));
            
        motor_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "motor_data", 10,
            std::bind(&CarOdometryNode::motor_callback, this, std::placeholders::_1));
        
        // Create timer for odometry publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&CarOdometryNode::publish_odometry, this));
        
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Initialize odometry message
        odom_msg_.header.frame_id = frame_id_;
        odom_msg_.child_frame_id = child_frame_id_;
        
        // Set covariance matrices
        set_covariance_matrices();
        
        RCLCPP_INFO(this->get_logger(), "Car Odometry Node started");
        RCLCPP_INFO(this->get_logger(), "Wheel radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "Wheelbase: %.3f m", wheelbase_);
        RCLCPP_INFO(this->get_logger(), "Track width: %.3f m", track_width_);
    }

private:
    // Parameters
    double wheel_radius_;
    double wheelbase_;
    double track_width_;
    double publish_rate_;
    std::string frame_id_;
    std::string child_frame_id_;
    
    // Odometry state
    double x_, y_, theta_;
    rclcpp::Time last_time_;
    
    // Latest sensor data
    sensor_msgs::msg::Imu latest_imu_;
    std_msgs::msg::Float32MultiArray latest_motor_;
    bool imu_received_ = false;
    bool motor_received_ = false;
    
    // ROS2 objects
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Odometry message
    nav_msgs::msg::Odometry odom_msg_;
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_ = *msg;
        imu_received_ = true;
    }
    
    void motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        latest_motor_ = *msg;
        motor_received_ = true;
    }
    
    void set_covariance_matrices()
    {
        // Position covariance (6x6 matrix)
        // [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        odom_msg_.pose.covariance[0] = 0.1;   // xx
        odom_msg_.pose.covariance[7] = 0.1;   // yy
        odom_msg_.pose.covariance[14] = 0.1;   // zz
        odom_msg_.pose.covariance[21] = 0.1;   // roll
        odom_msg_.pose.covariance[28] = 0.1;   // pitch
        odom_msg_.pose.covariance[35] = 0.1;   // yaw
        
        // Velocity covariance (6x6 matrix)
        odom_msg_.twist.covariance[0] = 0.1;   // vx
        odom_msg_.twist.covariance[7] = 0.1;   // vy
        odom_msg_.twist.covariance[14] = 0.1;  // vz
        odom_msg_.twist.covariance[21] = 0.1;  // wx
        odom_msg_.twist.covariance[28] = 0.1;  // wy
        odom_msg_.twist.covariance[35] = 0.1;  // wz
    }
    
    void publish_odometry()
    {
        if (!imu_received_ || !motor_received_) {
            return; // Don't publish if we don't have sensor data
        }
        
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0) {
            return; // Avoid division by zero
        }
        
        // Calculate velocities from motor data
        double right_rpm = latest_motor_.data[0];
        double left_rpm = latest_motor_.data[1];
        
        // Convert RPM to linear velocities
        double right_velocity = (right_rpm / 60.0) * 2.0 * M_PI * wheel_radius_;
        double left_velocity = (left_rpm / 60.0) * 2.0 * M_PI * wheel_radius_;
        
        // Calculate linear and angular velocities
        double linear_velocity = (right_velocity + left_velocity) / 2.0;
        double angular_velocity = (right_velocity - left_velocity) / track_width_;
        
        // Alternative: Use IMU for angular velocity (more accurate)
        if (imu_received_) {
            angular_velocity = latest_imu_.angular_velocity.z;
        }
        
        // Update position using integration
        x_ += linear_velocity * cos(theta_) * dt;
        y_ += linear_velocity * sin(theta_) * dt;
        theta_ += angular_velocity * dt;
        
        // Normalize angle
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        
        // Fill odometry message
        odom_msg_.header.stamp = current_time;
        
        // Position
        odom_msg_.pose.pose.position.x = x_;
        odom_msg_.pose.pose.position.y = y_;
        odom_msg_.pose.pose.position.z = 0.0;
        
        // Orientation (convert from yaw to quaternion)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg_.pose.pose.orientation = tf2::toMsg(q);
        
        // Linear velocity
        odom_msg_.twist.twist.linear.x = linear_velocity;
        odom_msg_.twist.twist.linear.y = 0.0;
        odom_msg_.twist.twist.linear.z = 0.0;
        
        // Angular velocity
        odom_msg_.twist.twist.angular.x = 0.0;
        odom_msg_.twist.twist.angular.y = 0.0;
        odom_msg_.twist.twist.angular.z = angular_velocity;
        
        // Publish odometry
        odom_publisher_->publish(odom_msg_);
        
        // Publish twist for debugging
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_velocity;
        twist_msg.angular.z = angular_velocity;
        twist_publisher_->publish(twist_msg);
        
        // Publish transform
        publish_transform(current_time);
        
        last_time_ = current_time;
    }
    
    void publish_transform(const rclcpp::Time& time)
    {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = time;
        transform.header.frame_id = frame_id_;
        transform.child_frame_id = child_frame_id_;
        
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
