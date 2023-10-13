#include "rclcpp/rclcpp.hpp"

#include "turtlefied_pkg//check_if_succeed.hpp"

/**
 * @brief
 */

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
using namespace std::chrono;
using namespace BT;

CheckGoal::CheckGoal(const std::string& name, const NodeConfiguration& config) 
        : StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("Goal_Check");
        sub_ = node_->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
            "navigate_to_pose/_action/feedback",
            10,
            std::bind(&CheckGoal::callback, this, std::placeholders::_1)
        );
        is_goal_succeed = false;
    }

//mandator to define this STATIC Method
PortsList CheckGoal::providedPorts(){return{};}

void CheckGoal::callback(const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr status_)
{
    // Callback logic when a new message arrives
    received_msg_ = *status_;
    eta = received_msg_.feedback.estimated_time_remaining;
    if(received_msg_.feedback.distance_remaining <= 0.1){
        // std::cout << "is_goal_succeed!!" << std::endl;
        is_goal_succeed = true;
    }

}

//Override the virtual fx onStart()
NodeStatus CheckGoal::onStart()
{
    return NodeStatus::RUNNING;
}

//Override the virtual fx onRunning()
NodeStatus CheckGoal::onRunning()
{  
    if(is_goal_succeed){
        std::cout << "Navigate Success" << std::endl;
        is_goal_succeed = false;
        return NodeStatus::SUCCESS;
    }
    // if (received_msg_.feedback.distance_remaining > 0.0){
    //     RCLCPP_INFO(node_->get_logger(), "Received goal: ETA in %d sec %d", 
    //             eta.sec, eta.nanosec);
    // }
    rclcpp::spin_some(node_);
    SleepMS(1000);
    return NodeStatus::RUNNING;
}

void CheckGoal::onHalted()
{
    std::cout << "CheckGoal interrupted!" << std::endl;
}