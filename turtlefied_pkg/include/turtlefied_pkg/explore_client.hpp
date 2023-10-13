#ifndef TURTLEFIED_PKG__SUBSCRIBER_EXAMPLE_HPP_
#define TURTLEFIED_PKG__SUBSCRIBER_EXAMPLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

/**
 * @brief 
 * *
 */

using namespace std::chrono;
using namespace BT;

class FetchGoal : public StatefulActionNode
{
public:
    FetchGoal(const std::string& name, const BT::NodeConfiguration& config);
    static PortsList providedPorts();
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;
    virtual void onHalted() override;
private:
    system_clock::time_point deadline_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscr_;
    geometry_msgs::msg::Point received_point_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    void callback(const geometry_msgs::msg::Point::SharedPtr msg);
    bool point_received_;
};

#endif //TURTLEFIED_PKG__SUBSCRIBER_EXAMPLE_HPP_