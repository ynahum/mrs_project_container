#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

using namespace BT;

class UpdatePlanFromTopic : public SyncActionNode
{
public:
  UpdatePlanFromTopic(const std::string& name, const NodeConfiguration& config)
  : SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("update_plan_from_topic_bt_node");
    RCLCPP_INFO(node_->get_logger(), "[UpdatePlanFromTopic] Constructor run!");
    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/plan", rclcpp::QoS(1).durability_volatile().reliable(),
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

  static PortsList providedPorts()
  {
    return { OutputPort<nav_msgs::msg::Path>("path") };
  }

  NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "Tick called");
    if (!path_available_) {
      RCLCPP_WARN(node_->get_logger(), "No path available yet");
      return NodeStatus::FAILURE;
    }

    RCLCPP_WARN(node_->get_logger(), "providedPorts: setOutput()");
    setOutput("path", latest_path_);

    return NodeStatus::RUNNING;
  }

private:
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path latest_path_;
  std::atomic<bool> path_available_{false};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::thread executor_thread_;
};

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<UpdatePlanFromTopic>("UpdatePlanFromTopic");
}
