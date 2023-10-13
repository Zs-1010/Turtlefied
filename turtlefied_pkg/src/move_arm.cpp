#include "turtlefied/move_arm.hpp"


using namespace BT;
using namespace std::chrono_literals;

MoveArm::MoveArm(const std::string& name, const NodeConfiguration& conf)
    : SyncActionNode(name, conf)
{
    arm_node_ = rclcpp::Node::make_shared("arm_bt_node");
    arm_client_ = arm_node_->create_client<modpi_ik_pkg::srv::SetArmPose>("/set_arm_pose");
    std::string arm_status;
    getInput("action", arm_status);
    if(arm_status=="enable")
    {
        arm_state = "DisinfectionState";
    }
    else if(arm_status=="disable")
    {
        arm_state = "ObjectDetectionState";
    }
}

NodeStatus MoveArm::tick()
{
    auto request = std::make_shared<modpi_ik_pkg::srv::SetArmPose::Request>();
    request->target_name = arm_state;
    while(!arm_client_->wait_for_service(1s)){
        if(!rclcpp::ok()){
            return NodeStatus::FAILURE;
        }
        RCLCPP_INFO(arm_node_->get_logger(), "waiting for service to become available..");
    }
    auto future_result = arm_client_->async_send_request(request);
    if(rclcpp::spin_until_future_complete(arm_node_, future_result) == 
        rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(arm_node_->get_logger(), "Success set arm to %s mode", arm_state.c_str());
            return NodeStatus::SUCCESS;
        }
    else{
        RCLCPP_INFO(arm_node_->get_logger(), "Failed to call service!");
        return NodeStatus::FAILURE;
    }
}