#ifndef TURTLEFIED_PKG__OBJ_BROADCASTER_HPP__
#define TURTLEFIED_PKG__OBJ_BROADCASTER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
 * @brief This Node will do object pose estimate to project in map
 * What I need: 
 */

using namespace BT;
class ObjBroadcaster : public SyncActionNode
{
public:
    ObjBroadcaster(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts(){return{};}
    virtual NodeStatus tick() override;
    // bool IsObjAlreadyExist(const geometry_msgs::msg::Pose& obj_pose_);
    // void broadcastobjecttf();

private:  
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string map_frame;
    std::string object_frame;
    int obj_num;
};

#endif //TURTLEFIED_PKG__OBJ_BROADCASTER_HPP__