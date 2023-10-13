#include <string>
#include "turtlefied/is_distance_reached.hpp"

using namespace BT;
    
IsDistanceReached::IsDistanceReached(
    const std::string& condition_name,
    const NodeConfiguration& conf)
: ConditionNode(condition_name, conf)
{
    node_ = rclcpp::Node::make_shared("is_distance_reached");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_exec_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    nav2_sub_ = node_->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
            "navigate_to_pose/_action/feedback",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsDistanceReached::nav2FeedbackCallback, this, std::placeholders::_1),
            sub_option);
    is_distance_reached = false;
    distance_travel = 0.0;
    feedback_msg_d = 0.0;

}

NodeStatus IsDistanceReached::tick()
{
    callback_group_exec_.spin_some();
    if(is_distance_reached){
        is_distance_reached = false;
        return NodeStatus::SUCCESS;
    }
    else{
        if(old_ > feedback_msg_d){
            distance_travel = old_ - feedback_msg_d;
        }
        else{
            old_ = feedback_msg_d;
            distance_travel = 0.0;
        }
    }
    return NodeStatus::FAILURE;
}

void IsDistanceReached::nav2FeedbackCallback(nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg)
{
    feedback_msg_d = msg->feedback.distance_remaining;
    if(distance_travel > distance_){
        is_distance_reached = true;
        distance_travel = 0.0;
        old_ = 0.0;
    }
}

