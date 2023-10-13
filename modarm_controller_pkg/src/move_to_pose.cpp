#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "modpi_custom_msgs/srv/move_arm.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/*This Node is for requesting a service that will move the arm on 
    a predefined position, currently this service supports 2 modes;
    1. Object Detection Mode = ObjectDetect_Mode
    2. Disinfection Mode = Disinfect_Mode
    use this command to request: 
        ros2 service call /set_arm_pose modpi_custom_msgs/srv/MoveArm "{position_name: 'ObjectDetect_Mode'}"
*/

using namespace std::placeholders;

class MoveArmToPoseServiceServer : public rclcpp::Node
{
public:
    MoveArmToPoseServiceServer() : Node("move_arm_to_pose")
    {
        service_ = create_service<modpi_custom_msgs::srv::MoveArm>("set_arm_pose",
                    std::bind(&MoveArmToPoseServiceServer::srvcallback, this, _1, _2));
        joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("joints_ctrl", 10);
        RCLCPP_INFO(rclcpp::get_logger("move_arm_to_pose"), "Ready to switch Mode");        
    }

private:
    rclcpp::Service<modpi_custom_msgs::srv::MoveArm>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    void srvcallback(const std::shared_ptr<modpi_custom_msgs::srv::MoveArm::Request> request_,
                    const std::shared_ptr<modpi_custom_msgs::srv::MoveArm::Response> response_
                    )
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_arm_to_pose"), "Request Pose Received "
                    << request_->position_name);
        sensor_msgs::msg::JointState joints_;
        if(request_->position_name == "Disinfect_Mode"){
            joints_.name = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
            joints_.position = {1.57, 3.14, 1.0, 3.14};
            joint_cmd_pub_->publish(joints_);
            response_->return_success = true;
            RCLCPP_INFO(rclcpp::get_logger("Success!"), "publish! Disinfect_Mode");
        }
        else if(request_->position_name == "ObjectDetect_Mode"){
            joints_.name = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
            joints_.position = {0.0, 0.0, 0.0, 0.0};
            joint_cmd_pub_->publish(joints_);
            response_->return_success = true;
            RCLCPP_INFO(rclcpp::get_logger("Success!"), "publish! ObjectDetect_Mode");
        }
        else{
            RCLCPP_ERROR(rclcpp::get_logger("Failed!"), "No Mode");
            response_->return_success = false;
        }
    }
};

int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<MoveArmToPoseServiceServer>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}