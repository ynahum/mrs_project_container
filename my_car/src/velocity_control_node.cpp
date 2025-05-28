#include "my_car/debug_logger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class VelocityController : public rclcpp::Node {
public:
    VelocityController() : Node("velocity_controller") {

        declare_parameter("enable_debug_speed", false);
        enable_debug_speed_ = get_parameter("enable_debug_speed").as_bool();
        RCLCPP_INFO(get_logger(), "enable_debug_speed_: %d", enable_debug_speed_);

        // Declare parameters with default values
        declare_parameter("Kp_th", 0.07);
        Kp_th_ = get_parameter("Kp_th").as_double();
        RCLCPP_INFO(get_logger(), "Kp_th_: %f", Kp_th_);

        declare_parameter("Kd_th", 0.0);
        Kd_th_ = get_parameter("Kd_th").as_double();
        RCLCPP_INFO(get_logger(), "Kd_th_: %f", Kd_th_);

        declare_parameter("Ki_th", 0.04);
        Ki_th_ = get_parameter("Ki_th").as_double();
        RCLCPP_INFO(get_logger(), "Ki_th_: %f", Ki_th_);

        declare_parameter("Kp_st", 0.6);
        Kp_st_ = get_parameter("Kp_st").as_double();
        RCLCPP_INFO(get_logger(), "Kp_st_: %f", Kp_st_);

        declare_parameter("Kd_st", 0.6);
        Kd_st_ = get_parameter("Kd_st").as_double();
        RCLCPP_INFO(get_logger(), "Kd_st_: %f", Kd_st_);

        declare_parameter("enable_debug_prints", false);
        enable_debug_prints_ = get_parameter("enable_debug_prints").as_bool();
        RCLCPP_INFO(get_logger(), "enable_debug_prints_: %d", enable_debug_prints_);


        if (enable_debug_prints_) {
            debug_logger_ = std::make_shared<DebugLogger>("vel_controller");
        }

        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1));
        if (enable_debug_prints_) {
            speed_debug_sub_ =  this->create_subscription<std_msgs::msg::Float32>(
                "/autodrive/f1tenth_1/speed", 10, std::bind(&VelocityController::speedDebugCallback, this, std::placeholders::_1));
        } else {
            left_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/autodrive/f1tenth_1/left_encoder", 10, std::bind(&VelocityController::leftEncoderCallback, this, std::placeholders::_1));
            right_encoder_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/autodrive/f1tenth_1/right_encoder", 10, std::bind(&VelocityController::rightEncoderCallback, this, std::placeholders::_1));
        }
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/autodrive/f1tenth_1/imu", 10, std::bind(&VelocityController::imuCallback, this, std::placeholders::_1));

        // Publishers
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/steering_command", 10);

        if (nullptr != debug_logger_) {
            debug_logger_->write("construct low level velocity controller done");
            debug_logger_->write("Kp_th_=",Kp_th_);
            debug_logger_->write("Kd_th_=",Kd_th_);
            debug_logger_->write("Ki_th_=",Ki_th_);
            debug_logger_->write("Kp_st_=",Kp_st_);
            debug_logger_->write("Kd_st_=",Kd_st_);
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        desired_velocity_ = msg->linear.x;
        desired_angular_velocity_ = msg->angular.z;
        updateThrottleAndSteering();
    }

    void speedDebugCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        left_wheel_speed_ = right_wheel_speed_ = msg->data;
        if (nullptr != debug_logger_)
           debug_logger_->write("V=",left_wheel_speed_);
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
        if (nullptr != debug_logger_)
           debug_logger_->write("W=",current_angular_velocity_);
        updateThrottleAndSteering();
    }

    double computeSteeringAngle(double yaw_rate, double linear_velocity) {
        if (std::abs(linear_velocity) < 1e-5) {
            return 0.0;  // Avoid division by zero, assume straight
        }
        return std::atan2(wb_ * yaw_rate, linear_velocity);
    }

    void updateThrottleAndSteering() {
        auto now = this->get_clock()->now();
        if (prev_time_.nanoseconds() == 0) {
            prev_time_ = now;
            return;  // skip first iteration
        }   
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;

        // Prevent division by 0
        if (dt == 0.0) return;

        // Compute current velocity as the average of left and right wheel speeds
        double current_velocity = (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        double velocity_error = desired_velocity_ - current_velocity;
        integral_vel_error_ += velocity_error * dt;
        double derivative = (velocity_error - prev_vel_error_) / dt;
        prev_vel_error_ = velocity_error;
        if (nullptr != debug_logger_) {
            debug_logger_->write(
                "dV=",desired_velocity_,
                " ,V=",current_velocity,
                " ,eV=", velocity_error);
        }

        // PID control
        double throttle_command = Kp_th_ * velocity_error +
                            Ki_th_ * integral_vel_error_ +
                            Kd_th_ * derivative;

        // Clamp throttle if needed
        throttle_command = std::clamp(throttle_command, -throttle_limit_, throttle_limit_);

        // Compute steering adjustment
        double delta_desired = computeSteeringAngle(desired_angular_velocity_, current_velocity);
        double delta_current = computeSteeringAngle(current_angular_velocity_, current_velocity);
        double angular_velocity_error = desired_angular_velocity_ - current_angular_velocity_;
        double delta_error = delta_desired - delta_current;
        //double steering_command = angular_velocity_error * kP_steering_;
        double steering_command = delta_error * Kp_st_ + delta_current;
        // Clamp steering if needed
        steering_command = std::clamp(steering_command, -steering_limit_, steering_limit_);

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
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_debug_sub_;
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
    double Kp_th_;
    double Kd_th_;
    double Ki_th_;

    // Internal state
    double prev_vel_error_ = 0.0;
    double integral_vel_error_ = 0.0;
    rclcpp::Time prev_time_;

    double Kp_st_;
    double Kd_st_;

    const double wb_{0.324};
    const double throttle_limit_{1.0};
    const double steering_limit_{0.5236};
    const double wheel_radius_{0.059};
    double rw_last_pos_ = 0.0;
    rclcpp::Time rw_last_time_;
    bool rw_first_ = true;
    double lw_last_pos_ = 0.0;
    rclcpp::Time lw_last_time_;
    bool lw_first_ = true;

    bool enable_debug_speed_{false};
    bool enable_debug_prints_{false};
    std::shared_ptr<DebugLogger> debug_logger_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityController>());
    rclcpp::shutdown();
    return 0;
}
