#ifndef TURTLEFIED_PKG__NAVIGATE_TO_OBJ_HPP_
#define TURTLEFIED_PKG__NAVIGATE_TO_OBJ_HPP_

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono;
using namespace BT;

class NavigateToObj : public StatefulActionNode
{
public:
    NavigateToObj(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts(){return{};}
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;
    virtual void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    system_clock::time_point deadline_;

    rclcpp::Subscription<nav2_msgs::action::FollowPath_FeedbackMessage>::SharedPtr fp_sub_;
    void monitorobject(const nav2_msgs::action::FollowPath_FeedbackMessage::SharedPtr data_);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    void monitorzed(const geometry_msgs::msg::Twist::SharedPtr zed_);
    float distance_tol = 0.005;
    double currx_pose, curry_pose;

    double fp_dist = 10.0;
    double fp_speed = 1.0;
    double fp_zed = 1.0;
    geometry_msgs::msg::Pose target_pose;
};
#endif //TURTLEFIED_PKG__NAVIGATE_TO_OBJ_HPP_