#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ServoControl : public rclcpp::Node
{
public:
    ServoControl() : Node("arm_control")
    {
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
                    "joints_ctrl", 10, std::bind(&ServoControl::moveCallback, this, _1));
        servopub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                    "arm_controller/joint_trajectory", 10);
        RCLCPP_INFO(get_logger(), "Control Node Started");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr servopub_;

    void moveCallback(const sensor_msgs::msg::JointState &msg) const
    {
        trajectory_msgs::msg::JointTrajectory arm_cmd;
        arm_cmd.joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};

        trajectory_msgs::msg::JointTrajectoryPoint arm_goal_pose;
        arm_goal_pose.positions.insert(arm_goal_pose.positions.end(),
                                 msg.position.begin(), msg.position.begin()+4);

        arm_cmd.points.push_back(arm_goal_pose);
        servopub_->publish(arm_cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}