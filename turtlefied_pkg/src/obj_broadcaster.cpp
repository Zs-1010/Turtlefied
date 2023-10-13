#include "turtlefied/obj_broadcaster.hpp"
#include "turtlefied/shared_pose.hpp"
#include "turtlefied/shared_objpose.hpp"
#include "turtlefied/shared_frame.hpp"

using namespace std::chrono_literals;

using namespace BT;

ObjBroadcaster::ObjBroadcaster(const std::string& name, const NodeConfiguration& conf)
    :   SyncActionNode(name, conf)
{
    node_ = rclcpp::Node::make_shared("transform_object_pose");
    node_->get_parameter("use_sim_time").get_parameter_value().get<bool>();
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("use_sim_time", true)};
    node_->set_parameters(all_new_parameters);

    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    obj_num = 0;
    map_frame = "map";
    object_frame = "object_frame";
    // object_frame = "object_frame_" + std::to_string(obj_num);
}
NodeStatus ObjBroadcaster::tick()
{
    geometry_msgs::msg::Pose ob_pose_ = SharedPose::getInstance().getPose();
    RCLCPP_INFO(node_->get_logger(), "Got Object Pose: (%.2f,%.2f,%.2f)",
                ob_pose_.position.x,
                ob_pose_.position.y,
                ob_pose_.position.z);

    //broadcast tf of the object
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->get_clock()->now();
    t.header.frame_id = "realsense_depth_optical_frame";
    t.child_frame_id = object_frame;
    t.transform.translation.x = ob_pose_.position.x;
    t.transform.translation.y = ob_pose_.position.y;
    t.transform.translation.z = ob_pose_.position.z;
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(node_->get_logger(), "End Broadcast %s tf..",object_frame.c_str());
    return BT::NodeStatus::SUCCESS;
}