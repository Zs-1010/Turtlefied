#include "rclcpp/rclcpp.hpp"
#include "turtlefied/robot_pause.hpp"

using namespace std::chrono_literals;
using namespace BT;

RobotPause::RobotPause(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{
    client_ = rclcpp::Node::make_shared("robot_pause");
    manage_client_ = client_->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
}

PortsList RobotPause::providedPorts()
{
    return{};
}
NodeStatus RobotPause::tick()
{
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = 1;

    while(!manage_client_->wait_for_service(5s)){
        if(!rclcpp::ok()){
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "service not available, waiting again...!!!!!" << std::endl;
    }
    auto future_result = manage_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_, future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // std::cout << "Robot will Pause!!!!!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    else{
            std::cout << "FAIL TO CALL SERVICE!!!!!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

    std::cout << "What happen?" << std::endl;
    return BT::NodeStatus::FAILURE;
}
