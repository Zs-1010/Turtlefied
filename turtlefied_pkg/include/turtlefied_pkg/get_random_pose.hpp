#ifndef TURTLEFIED_PKG__GET_RANDOM_POSE_HPP_
#define TURTLEFIED_PKG__GET_RANDOM_POSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "modpi_custom_msgs/srv/obj_coor.hpp"

using namespace BT;
using namespace std::chrono;

class GoToRandomPose : public StatefulActionNode
{
public:
    GoToRandomPose(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts(){return{};}
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;
    virtual void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_;
    void getrandposecallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_);
    std::vector<geometry_msgs::msg::PoseStamped> available_poses;

    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr nav_sub_;
    void checkrandposestatcallback(const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr stat_);
    double dist_;
    system_clock::time_point deadline_;
};

class EnableObjectDetection : public SyncActionNode
{
public:
    EnableObjectDetection(const std::string& name, const NodeConfiguration& conf);
    virtual NodeStatus tick() override;
    static PortsList providedPorts(){return{
    };}
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<modpi_custom_msgs::srv::ObjCoor>::SharedPtr ope_client;
};


class RotateInPlace : public SyncActionNode
{
public:
    RotateInPlace(const std::string& name, const NodeConfiguration& conf);
    virtual NodeStatus tick() override;
    static PortsList providedPorts(){return{
        InputPort<std::string>("direction", std::string("clockwise"), "Angular speed of the robot"),
    };}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::string direction_;
    double speed_;
};
#endif //TURTLEFIED_PKG__GET_RANDOM_POSE_HPP_