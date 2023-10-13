#ifndef TURTLEFIED_PKG__ROBOT_PAUSE_HPP_
#define TURTLEFIED_PKG__ROBOT_PAUSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class RobotPause : public SyncActionNode
{
public:
  RobotPause(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  virtual NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr manage_client_;
};

#endif //TURTLEFIED_PKG__ROBOT_PAUSE_HPP_