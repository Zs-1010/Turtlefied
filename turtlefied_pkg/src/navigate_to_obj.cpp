#include "turtlefied/navigate_to_obj.hpp"
#include "turtlefied/shared_objpose.hpp"
#include "turtlefied/shared_rejectedpose.hpp"


using namespace BT;
using namespace std::chrono_literals;

NavigateToObj::NavigateToObj(const std::string& name, const NodeConfiguration& conf)
    :   StatefulActionNode(name, conf)
{
    node_ = rclcpp::Node::make_shared("go_to_obj");
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10);
    fp_sub_ = node_->create_subscription<nav2_msgs::action::FollowPath_FeedbackMessage>(
            "/follow_path/_action/feedback",
            10,
            std::bind(&NavigateToObj::monitorobject, this, std::placeholders::_1)
        );

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&NavigateToObj::monitorzed, this, std::placeholders::_1)
        );    
    node_->get_parameter("use_sim_time").get_parameter_value().get<bool>();
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("use_sim_time", true)};
    node_->set_parameters(all_new_parameters);
}

void NavigateToObj::monitorzed(const geometry_msgs::msg::Twist::SharedPtr zed_)
{
    fp_zed = zed_->angular.z;
}

void NavigateToObj::monitorobject(const nav2_msgs::action::FollowPath_FeedbackMessage::SharedPtr data_)
{
    fp_dist = data_->feedback.distance_to_goal;
    fp_speed = data_->feedback.speed;
}

NodeStatus NavigateToObj::onStart()
{
    int msec = 60000; //1min
    deadline_ = system_clock::now() + milliseconds(msec);
    rclcpp::spin_some(node_);
    geometry_msgs::msg::Pose p = SharedObjPose::getInstance().getPose();
    geometry_msgs::msg::PoseStamped p_;
    p_.header.frame_id = "map";
    p_.header.stamp = node_->get_clock()->now();
    p_.pose.position.x = p.position.x;
    p_.pose.position.y = p.position.y;
    p_.pose.orientation.z = p.orientation.z;
    
    pub_->publish(p_);

    target_pose.position.x = p_.pose.position.x;
    target_pose.position.y = p_.pose.position.y;
    target_pose.orientation.z = p_.pose.orientation.z;

    RCLCPP_INFO(node_->get_logger(), "navigate to object: (%.2f, %.2f) θ:%.2f)",
       p_.pose.position.x, p_.pose.position.y, p_.pose.orientation.z);
    return NodeStatus::RUNNING;
}
NodeStatus NavigateToObj::onRunning()
{
    if(system_clock::now() >= deadline_){
        RCLCPP_INFO(node_->get_logger(), "Cannot Compute path to navigate through the object!");
        SharedRejectedPose::getInstance().setrejectedPose(target_pose);
        return NodeStatus::FAILURE;
    }
    rclcpp::spin_some(node_);
    if(fp_dist <= 0.28 && fp_speed == 0.0) //this should be set depending with the arm's reach=50cm
    {
        RCLCPP_INFO(node_->get_logger(), "Object is near!");
        SharedRejectedPose::getInstance().clear(); //pose will be added
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

void NavigateToObj::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Interrupted!");
}