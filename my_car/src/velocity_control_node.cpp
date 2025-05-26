#include "my_car/debug_logger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class VelocityController : public rclcpp::Node {
public:
    VelocityController() : Node("velocity_controller") {

        // Declare parameters with default values
        declare_parameter("kP_throttle", 0.04);
        declare_parameter("kD_throttle", 0.04);
        kP_throttle_ = get_parameter("kP_throttle").as_double();
        kD_throttle_ = get_parameter("kD_throttle").as_double();
        RCLCPP_INFO(get_logger(), "kP_throttle_: %f", kP_throttle_);
        RCLCPP_INFO(get_logger(), "kD_throttle_: %f", kD_throttle_);

        declare_parameter("kP_steering", 0.6);
        declare_parameter("kD_steering", 0.6);
        kP_steering_ = get_parameter("kP_steering").as_double();
        kD_steering_ = get_parameter("kD_steering").as_double();
        RCLCPP_INFO(get_logger(), "kP_steering_: %f", kP_steering_);
        RCLCPP_INFO(get_logger(), "kD_steering_: %f", kD_steering_);

        declare_parameter("enable_debug_prints", false);
        enable_debug_prints_ = get_parameter("enable_debug_prints").as_bool();
        RCLCPP_INFO(get_logger(), "enable_debug_prints_: %d", enable_debug_prints_);


        if (enable_debug_prints_) {
            debug_logger_ = std::make_shared<DebugLogger>("vel_controller");
        }

        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1));
        left_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/autodrive/f1tenth_1/left_encoder", 10, std::bind(&VelocityController::leftEncoderCallback, this, std::placeholders::_1));
        right_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
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

    void leftEncoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.empty()) return;

        double current_pos = msg->position[0];
        rclcpp::Time current_time = msg->header.stamp;

        if (!lw_first_) {
            double dt = (current_time - lw_last_time_).seconds();
            if (dt > 0.0) {
                double angular_velocity = (current_pos - lw_last_pos_) / dt;
                left_wheel_speed_ = angular_velocity * wheel_radius_;
                // if (nullptr != debug_logger_)
                //   debug_logger_->write("left_wheel_speed_=",left_wheel_speed_);
            }
            updateThrottleAndSteering();
        } else {
            lw_first_ = false;
        }

        lw_last_pos_ = current_pos;
        lw_last_time_ = current_time;
    }

    void rightEncoderCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {

        if (msg->position.empty()) return;

        double current_pos = msg->position[0];
        rclcpp::Time current_time = msg->header.stamp;

        if (!rw_first_) {
            double dt = (current_time - rw_last_time_).seconds();
            if (dt > 0.0) {
                double angular_velocity = (current_pos - rw_last_pos_) / dt;
                right_wheel_speed_ = angular_velocity * wheel_radius_;
                // if (nullptr != debug_logger_)
                //   debug_logger_->write("right_wheel_speed_=",right_wheel_speed_);
            }
            updateThrottleAndSteering();
        } else {
            rw_first_ = false;
        }

        rw_last_pos_ = current_pos;
        rw_last_time_ = current_time;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_angular_velocity_ = msg->angular_velocity.z;
        // if (nullptr != debug_logger_)
        //   debug_logger_->write("imuCallback: current_angular_velocity_=",current_angular_velocity_);
        updateThrottleAndSteering();
    }

    double computeSteeringAngle(double yaw_rate, double linear_velocity) {
        if (std::abs(linear_velocity) < 1e-5) {
            return 0.0;  // Avoid division by zero, assume straight
        }
        return std::atan2(wb_ * yaw_rate, linear_velocity);
    }

    void updateThrottleAndSteering() {
        // Compute current velocity as the average of left and right wheel speeds
        double current_velocity = (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        double velocity_error = desired_velocity_ - current_velocity;
        if (nullptr != debug_logger_) {
            debug_logger_->write(
                "v_w_l=",left_wheel_speed_,
                " ,v_w_r=",right_wheel_speed_);
            debug_logger_->write(
                "dV=",desired_velocity_,
                " ,V=",current_velocity,
                " ,eV=", velocity_error);
        }

        // Simple proportional controller for throttle
        double throttle_command = velocity_error * kP_throttle_;
        
        // Compute steering adjustment
        double delta_desired = computeSteeringAngle(desired_angular_velocity_, current_velocity);
        double delta_current = computeSteeringAngle(current_angular_velocity_, current_velocity);
        double angular_velocity_error = desired_angular_velocity_ - current_angular_velocity_;
        double delta_error = delta_desired - delta_current;
        //double steering_command = angular_velocity_error * kP_steering_;
        double steering_command = delta_error * kP_steering_ + delta_current;
        steering_command = std::max(-steering_limit_, steering_command);
        steering_command = std::min(steering_limit_, steering_command);

        if (nullptr != debug_logger_) {
            debug_logger_->write(
                "dW=",desired_angular_velocity_,
                " ,W=",current_angular_velocity_,
                " ,eW=", angular_velocity_error);
            debug_logger_->write(
                "dDelta=",delta_desired,
                " ,Delta=",delta_current,
                " ,eDelta=", delta_error);
            debug_logger_->write(
                "th_cmd=",throttle_command,
                " ,st_cmd=", steering_command);
        }

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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_encoder_sub_;
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
    double kP_throttle_;
    double kD_throttle_;

    double kP_steering_;
    double kD_steering_;

    const double wb_{0.324};
    const double steering_limit_{0.5236};
    const double wheel_radius_{0.059};
    double rw_last_pos_ = 0.0;
    rclcpp::Time rw_last_time_;
    bool rw_first_ = true;
    double lw_last_pos_ = 0.0;
    rclcpp::Time lw_last_time_;
    bool lw_first_ = true;

    bool enable_debug_prints_{false};
    std::shared_ptr<DebugLogger> debug_logger_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityController>());
    rclcpp::shutdown();
    return 0;
}
