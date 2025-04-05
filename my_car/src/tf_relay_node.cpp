#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h" 
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/convert.h"

class TfRelayNode : public rclcpp::Node
{
public:
    TfRelayNode() : Node("tf_relay_node")
    {
        // Create a TransformListener to listen for the original transform
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // Create a TransformBroadcaster to send the transformed transform
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer to periodically send transforms (e.g., every 0.1s)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TfRelayNode::broadcast_transform, this));
    }

private:
    void broadcast_transform()
    {
        try
        {
            // Get the transform from 'world' to 'f1tenth_1'
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "world", "f1tenth_1", rclcpp::Time(0));

            // Create a new transform to publish as 'map' -> 'f1tenth_1'
            geometry_msgs::msg::TransformStamped new_transform;
            new_transform.header.stamp = this->get_clock()->now();
            new_transform.header.frame_id = "map";  // New parent frame
            new_transform.child_frame_id = "f1tenth_1";

            // Copy translation and rotation from the original transform
            new_transform.transform.translation.x = transform.transform.translation.x;
            new_transform.transform.translation.y = transform.transform.translation.y;
            new_transform.transform.translation.z = transform.transform.translation.z;
            new_transform.transform.rotation = transform.transform.rotation;

            // Publish the new transform
            tf_broadcaster_->sendTransform(new_transform);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not relay transform: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfRelayNode>());
    rclcpp::shutdown();
    return 0;
}
