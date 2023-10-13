#ifndef TURTLEFIED_PKG__COMPUTE_OBJ_TO_POSE_HPP__
#define TURTLEFIED_PKG__COMPUTE_OBJ_TO_POSE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "builtin_interfaces/msg/time.hpp"

/**
 * @brief This Node will do object pose estimate to project in map
 * What I need: 
 */

using namespace BT;
class ComputeObjToPose : public StatefulActionNode
{
public:
    ComputeObjToPose(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts(){return{};}
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;
    virtual void onHalted() override;
    bool IsObjAlreadyExist(const geometry_msgs::msg::Pose& obj_pose_);

private:  
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string map_frame;
    std::string object_frame;
    int obj_num;
    geometry_msgs::msg::PoseArray object_poses_;
    float object_area_m = 0.01;
    geometry_msgs::msg::Pose ob_pose_;
    builtin_interfaces::msg::Time set_object_time_;
};

#endif //TURTLEFIED_PKG__COMPUTE_OBJ_TO_POSE_HPP__