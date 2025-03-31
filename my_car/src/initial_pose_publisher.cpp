#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

class InitialPosePublisher : public rclcpp::Node {
public:
    InitialPosePublisher() : Node("initial_pose_publisher"),
                             tf_buffer_(this->get_clock()),
                             tf_listener_(tf_buffer_) {

        RCLCPP_INFO(this->get_logger(), "create initial_pose_publisher");

        // Declare parameters
        this->declare_parameter("x", 0.0);
        this->declare_parameter("y", 0.0);
        this->declare_parameter("z", 0.0);
        this->declare_parameter("qx", 0.0);
        this->declare_parameter("qy", 0.0);
        this->declare_parameter("qz", 0.0);
        this->declare_parameter("qw", 1.0);

        // Get parameter values
        x_ = this->get_parameter("x").as_double();
        y_ = this->get_parameter("y").as_double();
        z_ = this->get_parameter("z").as_double();
        qx_ = this->get_parameter("qx").as_double();
        qy_ = this->get_parameter("qy").as_double();
        qz_ = this->get_parameter("qz").as_double();
        qw_ = this->get_parameter("qw").as_double();

        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // TF timer (checks transform every 500ms)
        tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(300),
            std::bind(&InitialPosePublisher::checkTransformAndPublish, this));
    }

private:
    void checkTransformAndPublish() {
        std::string base_frame = "map";
        std::string target_frame = "f1tenth_1";
        try {
            // Check if transform exists
            tf_buffer_.lookupTransform(target_frame, base_frame, tf2::TimePointZero);

            // Transform is available, publish initial pose
            publishInitialPose();
            tf_timer_->cancel(); // Stop checking after publishing

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Waiting for transform '%s' -> '%s': %s", target_frame.c_str(), base_frame.c_str(), ex.what());
        }
    }

    void publishInitialPose() {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = x_;
        pose_msg.pose.pose.position.y = y_;
        pose_msg.pose.pose.position.z = z_;
        pose_msg.pose.pose.orientation.x = qx_;
        pose_msg.pose.pose.orientation.y = qy_;
        pose_msg.pose.pose.orientation.z = qz_;
        pose_msg.pose.pose.orientation.w = qw_;

        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose: [%.2f, %.2f, %.2f] | Quaternion [%.2f, %.2f, %.2f, %.2f]",
                    x_, y_, z_, qx_, qy_, qz_, qw_);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double x_, y_, z_;
    double qx_, qy_, qz_, qw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
