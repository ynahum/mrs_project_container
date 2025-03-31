#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

class VelocityController : public rclcpp::Node {
public:
    VelocityController() : Node("velocity_controller") {
        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1));
        left_encoder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/left_encoder", 10, std::bind(&VelocityController::leftEncoderCallback, this, std::placeholders::_1));
        right_encoder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/right_encoder", 10, std::bind(&VelocityController::rightEncoderCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10, std::bind(&VelocityController::imuCallback, this, std::placeholders::_1));

        // Publishers
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        desired_velocity_ = msg->linear.x;
        desired_angular_velocity_ = msg->angular.z;
        updateThrottleAndSteering();
    }

    void leftEncoderCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        left_wheel_speed_ = msg->data;
        updateThrottleAndSteering();
    }

    void rightEncoderCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        right_wheel_speed_ = msg->data;
        updateThrottleAndSteering();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_angular_velocity_ = msg->angular_velocity.z;
        updateThrottleAndSteering();
    }

    void updateThrottleAndSteering() {
        // Compute current velocity as the average of left and right wheel speeds
        double current_velocity = (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        double velocity_error = desired_velocity_ - current_velocity;
        
        // Simple proportional controller for throttle
        double throttle_command = velocity_error * kP_throttle_;
        
        // Compute steering adjustment
        double angular_error = desired_angular_velocity_ - current_angular_velocity_;
        double steering_command = angular_error * kP_steering_;
        
        // Publish throttle command
        auto throttle_msg = std_msgs::msg::Float32();
        throttle_msg.data = throttle_command;
        throttle_pub_->publish(throttle_msg);

        // Publish steering command
        auto steering_msg = std_msgs::msg::Float32();
        steering_msg.data = steering_command;
        steering_pub_->publish(steering_msg);
    }

    // ROS 2 subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // ROS 2 publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;

    // Control variables
    double desired_velocity_ = 0.0;
    double desired_angular_velocity_ = 0.0;
    double left_wheel_speed_ = 0.0;
    double right_wheel_speed_ = 0.0;
    double current_angular_velocity_ = 0.0;
    
    // Proportional gains
    const double kP_throttle_ = 0.04;
    const double kP_steering_ = 0.2;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityController>());
    rclcpp::shutdown();
    return 0;
}
