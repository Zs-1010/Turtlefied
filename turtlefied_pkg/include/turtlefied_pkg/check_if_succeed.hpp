#ifndef TURTLEFIED_PKG__CHECK_IF_SUCCEED_HPP_
#define TURTLEFIED_PKG__CHECK_IF_SUCCEED_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <builtin_interfaces/msg/duration.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

/**
 * @brief 
 * *
 */

using namespace std::chrono;
using namespace BT;

class CheckGoal : public StatefulActionNode
{
public:
    CheckGoal(const std::string& name, const BT::NodeConfiguration& config);
    static PortsList providedPorts();
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;
    virtual void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr sub_;
    nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage received_msg_;
    void callback(const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr status_);
    bool is_goal_succeed;
    builtin_interfaces::msg::Duration eta; 
};

#endif //TURTLEFIED_PKG__CHECK_IF_SUCCEED_HPP_