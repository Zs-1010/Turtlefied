#ifndef TURTLEFIED_PKG__MOVE_ARM_HPP_
#define TURTLEFIED_PKG__MOVE_ARM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "modpi_ik_pkg/srv/set_arm_pose.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
class MoveArm : public SyncActionNode
{
public:
    MoveArm(const std::string& name, const NodeConfiguration& conf);
    virtual NodeStatus tick() override;
    static PortsList providedPorts(){
        return{
            InputPort<std::string>("action", "disable", "Arm State [enable/disable]")
        };
    }
private:
    rclcpp::Node::SharedPtr arm_node_;
    rclcpp::Client<modpi_ik_pkg::srv::SetArmPose>::SharedPtr arm_client_;
    std::string arm_state;
};


#endif //TURTLEFIED_PKG__MOVE_ARM_HPP_