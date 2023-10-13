#include "rclcpp/rclcpp.hpp"
#include "turtlefied/robot_resume.hpp"

using namespace std::chrono_literals;
using namespace BT;

RobotResume::RobotResume(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
{
    client_ = rclcpp::Node::make_shared("robot_resume");
    manage_client_ = client_->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
}

PortsList RobotResume::providedPorts()
{
    return{};
}
NodeStatus RobotResume::tick()
{
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    if(request->command == 2){
        std::cout << "NavStack is alraedy running" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    request->command = 2;
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
            // std::cout << "Robot will Resume!!!!!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    else{
            std::cout << "FAIL TO CALL SERVICE!!!!!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

    std::cout << "What happen?" << std::endl;
    return BT::NodeStatus::FAILURE;
}
