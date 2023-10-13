#include <string>
#include <memory>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "turtlefied/is_return_to_origin.hpp"
#include "turtlefied/shared_data_storage.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace BT;
    
IsReturnToOrigin::IsReturnToOrigin(
    const std::string& condition_name,
    const NodeConfiguration& conf)
: ConditionNode(condition_name, conf),
    initialized_(false),
    global_frame_("map"),
    robot_base_frame_("base_footprint")
{
    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);
}

IsReturnToOrigin::~IsReturnToOrigin()
{
    cleanup();
}

NodeStatus IsReturnToOrigin::tick()
{
    if (!initialized_) {
        initialize();
    }

    if (IsReturnSuccessful()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void IsReturnToOrigin::initialize()
{
    node_ = rclcpp::Node::make_shared("Is_return_to_origin");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initialized_ = true;
    goal_reached_tol_ = 0.25;
}

bool IsReturnToOrigin::IsReturnSuccessful()
{
    // Retrieve the origin_pose from the shared data storage
    geometry_msgs::msg::Pose2D origin_pose = SharedDataStorage::getInstance().getOriginPose();
    RCLCPP_INFO(node_->get_logger(), "Target Pose: %.1f,%.1f,%.1f ", origin_pose.x, origin_pose.y, origin_pose.theta);
    geometry_msgs::msg::TransformStamped t;
    try{
        t = tf_buffer_->lookupTransform(
            global_frame_, robot_base_frame_, tf2::TimePointZero
        ); // tf2::TimePointZero() will just get us the latest available transform
    }
    catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            node_->get_logger(), "Could not transform %s to %s: %s",
            robot_base_frame_.c_str(), global_frame_.c_str(), ex.what());
          return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Current Pose: %.2f,%.2f,%.2f ", t.transform.translation.x,
        t.transform.translation.y,
        t.transform.rotation.z);

    double dx = origin_pose.x - t.transform.translation.x;
    double dy = origin_pose.y - t.transform.translation.y;
    return (dx * dx + dy * dy) <= (goal_reached_tol_); //goal_tolerance
}