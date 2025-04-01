#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

using namespace std_msgs::msg;
using namespace sensor_msgs::msg;

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0) {
        // Subscribers
        left_encoder_sub_ = this->create_subscription<JointState>(
            "/autodrive/f1tenth_1/left_encoder", 100,
            std::bind(&OdometryNode::leftEncoderCallback, this, std::placeholders::_1));
        right_encoder_sub_ = this->create_subscription<JointState>(
            "/autodrive/f1tenth_1/right_encoder", 100,
            std::bind(&OdometryNode::rightEncoderCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<Imu>(
            "/autodrive/f1tenth_1/imu", 100,
            std::bind(&OdometryNode::imuCallback, this, std::placeholders::_1));

        // Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(get_logger(), "created odometry node");
        left_wheel_prev_time_ = right_wheel_prev_time_ = this->now();
    }

private:
    void leftEncoderCallback(const JointState::SharedPtr msg) {
        double current_position = msg->position[0];
        rclcpp::Time current_time = this->now();
        double dt = (current_time - left_wheel_prev_time_).seconds();
        left_wheel_speed_ = WHEEL_RADIUS_ * (current_position - left_wheel_prev_position_) / dt;
        updateOdometry();
        left_wheel_prev_time_ = current_time;
        left_wheel_prev_position_ = current_position;
    }

    void rightEncoderCallback(const JointState::SharedPtr msg) {
        double current_position = msg->position[0];
        rclcpp::Time current_time = this->now();
        double dt = (current_time - right_wheel_prev_time_).seconds();
        right_wheel_speed_ = WHEEL_RADIUS_ * (current_position - right_wheel_prev_position_) / dt;
        updateOdometry();
        right_wheel_prev_time_ = current_time;
        right_wheel_prev_position_ = current_position;
    }

    void imuCallback(const Imu::SharedPtr msg) {
        angular_velocity_ = msg->angular_velocity.z;
    }

    void updateOdometry() {
        double current_time = this->now().seconds();
        double dt = current_time - last_time_;
        last_time_ = current_time;

        double linear_velocity = (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        theta_ += angular_velocity_ * dt;
        x_ += linear_velocity * cos(theta_) * dt;
        y_ += linear_velocity * sin(theta_) * dt;

        //RCLCPP_INFO(get_logger(), "call publishOdometry");
        publishOdometry();

        //RCLCPP_INFO(get_logger(), "call publishTransform");
        publishTransform();
    }

    void publishOdometry() {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "f1tenth_1";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.twist.twist.linear.x = (left_wheel_speed_ + right_wheel_speed_) / 2.0;
        odom_msg.twist.twist.angular.z = angular_velocity_;

        odom_pub_->publish(odom_msg);
    }

    void publishTransform() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "f1tenth_1";
        
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.z = sin(theta_ / 2.0);
        transform.transform.rotation.w = cos(theta_ / 2.0);

        tf_broadcaster_->sendTransform(transform);
    }

    // ROS 2 entities
    rclcpp::Subscription<JointState>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<JointState>::SharedPtr right_encoder_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Odometry variables
    double x_, y_, theta_;
    rclcpp::Time left_wheel_prev_time_;
    double left_wheel_prev_position_ = 0.0;
    double left_wheel_speed_ = 0.0;
    rclcpp::Time right_wheel_prev_time_;
    double right_wheel_prev_position_ = 0.0;
    double right_wheel_speed_ = 0.0;
    double angular_velocity_ = 0.0;
    double last_time_ = 0.0;
    const float WHEEL_RADIUS_ = 0.059;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
