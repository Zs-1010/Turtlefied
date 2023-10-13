#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "example_interfaces/srv/trigger.hpp"
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
        service_ = create_service<example_interfaces::srv::Trigger>("set_arm_pose",
                    std::bind(&MoveArmToPoseServiceServer::srvcallback, this, _1, _2));
        joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("joints_ctrl", 10);
        RCLCPP_INFO(rclcpp::get_logger("move_arm_to_pose"), "Ready to switch Mode");        
    }

private:
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    void srvcallback(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request_,
                    const std::shared_ptr<example_interfaces::srv::Trigger::Response> response_
                    )
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("move_arm_to_pose"), "Request Pose Received "
                    );
        response_->success = true;
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