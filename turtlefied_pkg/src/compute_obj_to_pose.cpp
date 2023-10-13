#include "turtlefied_pkg//compute_obj_to_pose.hpp"
#include "turtlefied_pkg//shared_pose.hpp"
#include "turtlefied_pkg//shared_objpose.hpp"
#include "turtlefied_pkg//shared_rejectedpose.hpp"
using namespace std::chrono_literals;

using namespace BT;

ComputeObjToPose::ComputeObjToPose(const std::string& name, const NodeConfiguration& conf)
    :   StatefulActionNode(name, conf)
{
    node_ = rclcpp::Node::make_shared("compute_object_to_pose");
    node_->get_parameter("use_sim_time").get_parameter_value().get<bool>();
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("use_sim_time", true)};
    node_->set_parameters(all_new_parameters);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    obj_num = 0;
    map_frame = "map";
    object_frame = "object_frame";

}
NodeStatus ComputeObjToPose::onStart()
{
    ob_pose_ = SharedPose::getInstance().getPose();
    RCLCPP_INFO(node_->get_logger(), "Got Object Pose: (%.2f,%.2f,%.2f)",
                ob_pose_.position.x,
                ob_pose_.position.y,
                ob_pose_.position.z);

     //get the timestamped
    geometry_msgs::msg::TransformStamped get_timestamp;
    try{
        get_timestamp = tf_buffer_->lookupTransform(
            map_frame.c_str(), "realsense_depth_optical_frame", tf2::TimePointZero
        );
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            node_->get_logger(), "Could not transform %s to %s: %s",
            map_frame.c_str(), "realsense_depth_optical_frame", ex.what());
        return NodeStatus::FAILURE;
    }
    set_object_time_ = get_timestamp.header.stamp;
    return NodeStatus::RUNNING;
}
NodeStatus ComputeObjToPose::onRunning()
{
    //broadcast tf of the object
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = set_object_time_; //node_->get_clock()->now();
    t.header.frame_id = "realsense_depth_optical_frame";
    t.child_frame_id = "object_frame"; //object_frame;
    t.transform.translation.x = ob_pose_.position.x;
    t.transform.translation.y = ob_pose_.position.y;
    t.transform.translation.z = ob_pose_.position.z;
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(node_->get_logger(), "End Broadcast %s tf..",object_frame.c_str());

    //listen to
    geometry_msgs::msg::TransformStamped obj_tf;
    try{
        obj_tf = tf_buffer_->lookupTransform(
            map_frame.c_str(), object_frame.c_str(), tf2::TimePointZero, 50ms
        ); // tf2::TimePointZero() will just get us the latest available transform
    }

    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            node_->get_logger(), "Could not transform %s to %s: %s",
            map_frame.c_str(), object_frame.c_str(), ex.what());
        return NodeStatus::RUNNING;
    }
    
    RCLCPP_INFO(node_->get_logger(), " %s Pose in Map: (%.2f,%.2f, θ:%.2f)",
            object_frame.c_str(),
            obj_tf.transform.translation.x,
            obj_tf.transform.translation.y,
            obj_tf.transform.rotation.z + 0.785);

    geometry_msgs::msg::Pose p_;
    p_.position.x = obj_tf.transform.translation.x,
    p_.position.y = obj_tf.transform.translation.y,
    p_.orientation.z = obj_tf.transform.rotation.z + 0.785; //normalize orientation
    SharedObjPose::getInstance().setPose(p_);
    if(IsObjAlreadyExist(p_))
    {
        return NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

void ComputeObjToPose::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Interrupted!");
}

bool ComputeObjToPose::IsObjAlreadyExist(const geometry_msgs::msg::Pose& obj_pose_)
{
    if(object_poses_.poses.empty())
    {
        object_poses_.poses.push_back(obj_pose_);
        RCLCPP_INFO(node_->get_logger(), "Adding Object for the first time");
        return false;
    }

    // Updating the List
    RCLCPP_INFO(node_->get_logger(), "Updating List!");
    geometry_msgs::msg::Pose rejected_ = SharedRejectedPose::getInstance().getrejectedPose();
    if(!rejected_.position.x == 0 && !rejected_.position.y == 0)
    {
        for (auto it = object_poses_.poses.begin(); it != object_poses_.poses.end(); ++it) {
            // Compare the current pose (it) with the pose you want to remove (r_)
            if (*it == rejected_) {
                object_poses_.poses.erase(it);
                RCLCPP_INFO(node_->get_logger(), "removed!! (%.2f,%.2f) θ:%.2f"
                    ,rejected_.position.x
                    ,rejected_.position.y
                    ,rejected_.orientation.z
                    );
                break;  // Assuming there is only one matching pose, you can exit the loop
            }
        }
    }
    RCLCPP_INFO(node_->get_logger(), "List Updated!");

    for (const auto& pose_rcv : object_poses_.poses) 
    {
        RCLCPP_INFO(node_->get_logger(), "Validating if pose is within (%f,%f) : %ld total", 
            pose_rcv.position.x,
            pose_rcv.position.y,
            object_poses_.poses.size());

        // Calculate the limits for the perimeter
        double x_lower_limit = pose_rcv.position.x - object_area_m;
        double x_upper_limit = pose_rcv.position.x + object_area_m;
        double y_lower_limit = pose_rcv.position.y - object_area_m;
        double y_upper_limit = pose_rcv.position.y + object_area_m;
        RCLCPP_INFO(node_->get_logger(), "limits: x(%f,%f) :y(%f,%f)", 
            x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit
        );

        if(obj_pose_.position.x >= x_lower_limit &&
            obj_pose_.position.x <= x_upper_limit &&
            obj_pose_.position.y >= y_lower_limit &&
            obj_pose_.position.y <= y_upper_limit)
        {
            RCLCPP_INFO(node_->get_logger(), "Pose already exist, skip targeted disinfection!");
            return true;
        }
    }
    object_poses_.poses.push_back(obj_pose_);
    RCLCPP_INFO(node_->get_logger(), "New Object detected..add");
    return false;
}