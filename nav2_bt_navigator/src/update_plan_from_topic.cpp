#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"


namespace my_nav2_behavior_tree
{

class UpdatePlanFromTopic : public BT::ActionNodeBase
{
public:
  UpdatePlanFromTopic(const std::string& name,
  const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
  {
    node_ = rclcpp::Node::make_shared("update_plan_from_topic_bt_node");
    // RCLCPP_INFO(node_->get_logger(), "[UpdatePlanFromTopic] Constructor run!");
    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1).durability_volatile().reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
        path_available_ = true;
      });

    executor_thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });
  }

  ~UpdatePlanFromTopic()
  {
    rclcpp::shutdown();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<nav_msgs::msg::Path>("path") };
  }

  BT::NodeStatus tick() override
  {
    // RCLCPP_INFO(node_->get_logger(), "Tick called");
    if (!path_available_) {
      RCLCPP_WARN(node_->get_logger(), "No path available yet");
      return BT::NodeStatus::FAILURE;
    }

    // RCLCPP_WARN(node_->get_logger(), "providedPorts: setOutput()");
    setOutput("path", latest_path_);

    return BT::NodeStatus::SUCCESS;
  }

private:
  void halt() override {}
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path latest_path_;
  std::atomic<bool> path_available_{false};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::thread executor_thread_;
};
} // namespace nav2_behavior_tree

#include <behaviortree_cpp_v3/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<my_nav2_behavior_tree::UpdatePlanFromTopic>("UpdatePlanFromTopic");
}
