#ifndef TURTLEFIED_PKG__IS_RETURN_TO_ORIGIN_HPP_
#define TURTLEFIED_PKG__IS_RETURN_TO_ORIGIN_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
/**
 * @brief A BT::ConditionNode that check if current pose is in 
 * initial pose or got back to origin
 */
class IsReturnToOrigin : public ConditionNode
{
public:
    IsReturnToOrigin(const std::string& condition_name, const NodeConfiguration& conf);
    NodeStatus tick() override;
    IsReturnToOrigin() = delete;
    ~IsReturnToOrigin() override;
    void initialize();
    bool IsReturnSuccessful();
    static PortsList providedPorts(){
        return {
            InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
            InputPort<std::string>("robot_base_frame", std::string("base_footprint"), "Robot base frame")
        };
    }
protected:
    void cleanup(){}

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool initialized_;
    double goal_reached_tol_;
    std::string global_frame_;
    std::string robot_base_frame_;
};

#endif //TURTLEFIED_PKG__IS_RETURN_TO_ORIGIN_HPP_