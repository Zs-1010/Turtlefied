#ifndef TURTLEFIED_PKG__RETURN_TO_ORIG_HPP_
#define TURTLEFIED_PKG__RETURN_TO_ORIG_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class ReturnToOrig : public SyncActionNode
{
public:
  ReturnToOrig(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  virtual NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
  std_msgs::msg::Bool pause_exploration;
};

#endif //TURTLEFIED_PKG__RETURN_TO_ORIG_HPP_