#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <sstream>
#include <cmath>

class PathPublisherNode : public rclcpp::Node {
public:
    PathPublisherNode() : Node("path_publisher_node") {
        std::string csv_path = this->declare_parameter<std::string>("csv_path", "/home/dev/ros_ws/src/my_car/maps/practice_2_w_as_o_centerline.csv");
        lookahead_ = this->declare_parameter<int>("lookahead", 300);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PathPublisherNode::timerCallback, this));

        loadPathFromCSV(csv_path);
    }

private:
    void loadPathFromCSV(const std::string &file_path) {
        std::ifstream file(file_path);
        std::string line;
        // to ignore the first line: x,y
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            double x, y;
            char comma;
            if (ss >> x >> comma >> y) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.orientation.w = 1.0;
                path_points_.push_back(pose);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %lu path points", path_points_.size());
    }

    std::optional<geometry_msgs::msg::PoseStamped> getCurrentPose() {
        geometry_msgs::msg::PoseStamped pose;
        try {
            auto transform = tf_buffer_->lookupTransform(frame_id_, base_frame_, tf2::TimePointZero);
            pose.header = transform.header;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.orientation = transform.transform.rotation;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return std::nullopt;
        }
        return pose;
    }

    size_t findClosestIndex(const geometry_msgs::msg::PoseStamped &pose) {
        double min_dist = std::numeric_limits<double>::max();
        size_t index = 0;
        for (size_t i = 0; i < path_points_.size(); ++i) {
            double dx = path_points_[i].pose.position.x - pose.pose.position.x;
            double dy = path_points_[i].pose.position.y - pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                index = i;
            }
        }
        return index;
    }

    void timerCallback() {
        std::optional<geometry_msgs::msg::PoseStamped> opt_pose =
            getCurrentPose();
        if (opt_pose == std::nullopt)
            return;
        auto pose = opt_pose.value();
        size_t start_idx = findClosestIndex(pose);

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = frame_id_;
        path_msg.header.stamp = this->get_clock()->now();

        for (int i = 0; i < lookahead_; ++i) {
            size_t idx = (start_idx + i) % path_points_.size();
            path_msg.poses.push_back(path_points_[idx]);
        }
        // RCLCPP_INFO(this->get_logger(), "publish path");
        path_pub_->publish(path_msg);
    }

    std::vector<geometry_msgs::msg::PoseStamped> path_points_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string frame_id_ = "map";
    std::string base_frame_ = "f1tenth_1";
    int lookahead_ = 300;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
