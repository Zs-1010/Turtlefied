#include "rclcpp/rclcpp.hpp"
#include "turtlefied/return_to_orig.hpp"
#include "turtlefied/using_generic_types.hpp"
#include "turtlefied/shared_data_storage.hpp"

using namespace BT;

ReturnToOrig::ReturnToOrig(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("return_to_orig");
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10);
    explore_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/explore/resume",
        10);
    pause_exploration.data = true;
}

PortsList ReturnToOrig::providedPorts()
{
    return{InputPort<Pose2D>("origin_pose")};
}

NodeStatus ReturnToOrig::tick()
{
    explore_pub_->publish(pause_exploration);
    Pose2D pose;
    if ( !getInput<Pose2D>("origin_pose", pose))
    {
        throw BT::RuntimeError("missing required input [origin_pose]");
    }
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.orientation.z = pose.yaw;
    pub_->publish(pose_msg);

    geometry_msgs::msg::Pose2D pose_;
    pose_.x = pose.x;
    pose_.y = pose.y;
    pose_.theta = pose.yaw;
    SharedDataStorage::getInstance().setOriginPose(pose_);
    
    return NodeStatus::SUCCESS;
}
