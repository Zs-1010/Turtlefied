#ifndef TURTLEFIED_PKG__IS_DISTANCE_REACHED_HPP_
#define TURTLEFIED_PKG__IS_DISTANCE_REACHED_HPP_

#include <string>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
/**
 * @brief A BT::ConditionNode that listens to a topic and
 * returns SUCCESS when a certain distance is reached and FAILURE otherwise
 */
class IsDistanceReached : public ConditionNode
{
public:
    IsDistanceReached(const std::string& condition_name, const NodeConfiguration& conf);
    virtual NodeStatus tick() override;
    IsDistanceReached() = delete;
    static PortsList providedPorts(){
        return {
        InputPort<std::string>("distance_to_reach", "Target Distance to reach" ),
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_exec_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr nav2_sub_;

    void nav2FeedbackCallback(nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg);
    double feedback_msg_d;
    double distance_;    
    bool is_distance_reached;
    double distance_travel;
    double old_;
};

#endif //TURTLEFIED_PKG__IS_DISTANCE_REACHED_HPP_