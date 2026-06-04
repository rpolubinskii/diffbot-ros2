#if __has_include(<behaviortree_cpp/action_node.h>)
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#else
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#endif

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <mutex>
#include <string>

namespace diffbot
{

class WaitForFreshOdom : public BT::StatefulActionNode
{
public:
  WaitForFreshOdom(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");
    logger_ = node_->get_logger().get_child("WaitForFreshOdom");

    if (!getInput("odom_topic", odom_topic_)) {
      odom_topic_ = "/rtabmap/icp_odom";
    }
    getInput("timeout_sec", timeout_sec_);
    getInput("freshness_sec", freshness_sec_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_odom_receipt_time_ = node_->now();
        has_odom_ = true;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "odom_topic", "/rtabmap/icp_odom", "Odometry topic that must publish after the goal starts"),
      BT::InputPort<double>("timeout_sec", 8.0, "Seconds to wait before failing"),
      BT::InputPort<double>("freshness_sec", 0.5, "Maximum age of the received odometry")
    };
  }

  BT::NodeStatus onStart() override
  {
    start_time_ = node_->now();
    logged_waiting_ = false;
    return check_odom();
  }

  BT::NodeStatus onRunning() override
  {
    return check_odom();
  }

  void onHalted() override {}

private:
  BT::NodeStatus check_odom()
  {
    const auto now = node_->now();

    rclcpp::Time last_odom_time(0, 0, node_->get_clock()->get_clock_type());
    bool has_odom = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_odom = has_odom_;
      last_odom_time = last_odom_receipt_time_;
    }

    if (has_odom && last_odom_time >= start_time_ && (now - last_odom_time).seconds() <= freshness_sec_) {
      RCLCPP_INFO(
        logger_, "Fresh odometry received on %s after %.3f seconds",
        odom_topic_.c_str(), (now - start_time_).seconds());
      return BT::NodeStatus::SUCCESS;
    }

    const double elapsed_sec = (now - start_time_).seconds();
    if (elapsed_sec >= timeout_sec_) {
      RCLCPP_ERROR(
        logger_, "Timed out after %.3f seconds waiting for fresh odometry on %s",
        timeout_sec_, odom_topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!logged_waiting_) {
      RCLCPP_INFO(
        logger_, "Waiting up to %.3f seconds for fresh odometry on %s",
        timeout_sec_, odom_topic_.c_str());
      logged_waiting_ = true;
    }

    return BT::NodeStatus::RUNNING;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("WaitForFreshOdom")};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex mutex_;
  bool has_odom_{false};
  rclcpp::Time last_odom_receipt_time_{0, 0, RCL_ROS_TIME};

  std::string odom_topic_{"/rtabmap/icp_odom"};
  double timeout_sec_{8.0};
  double freshness_sec_{0.5};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  bool logged_waiting_{false};
};

}  // namespace diffbot

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<diffbot::WaitForFreshOdom>("WaitForFreshOdom");
}
