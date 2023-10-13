#include "turtlefied/is_validate_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono;
using namespace BT;

IsPoseExists::IsPoseExists(
    const std::string& condition_name, 
    const NodeConfiguration& conf)
: ConditionNode(condition_name, conf)
    ,initialized_(false)
    ,global_frame_("map")
    ,robot_base_frame_("base_footprint")
{
    getInput("global_frame", global_frame_);
    getInput("source_frame", robot_base_frame_);
    getInput("coverage", spray_rad_m);
    getInput("timeout", msec);
}

IsPoseExists::~IsPoseExists()
{
    cleanup();
}

NodeStatus IsPoseExists::tick()
{
    if(!initialized_){
        initialize();
    }
    rclcpp::spin_some(node_);

    deadline_ = system_clock::now() + milliseconds(msec);
    geometry_msgs::msg::TransformStamped t;
    while(system_clock::now() <= deadline_)
    {
        try{
            t = tf_buffer_->lookupTransform(
                global_frame_, robot_base_frame_, tf2::TimePointZero);
            RCLCPP_INFO(node_->get_logger(), "Got Tranform!!!");
            break;
        }
        catch(const tf2::TransformException &ex) {
            RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s",
            global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
            return BT::NodeStatus::FAILURE;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Current Pose: (x:%.2f,y:%.2f,z(yaw):%.2f)", 
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.rotation.z);
    
    currentpose_.position.x = t.transform.translation.x;
    currentpose_.position.y = t.transform.translation.y;
    if (IsPoseThere()) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void IsPoseExists::initialize()
{
    node_ = rclcpp::Node::make_shared("get_pose_in_map");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    initialized_ = true;
}

bool IsPoseExists::IsPoseThere()
{
    if(received_poses_.poses.empty())
    {
        received_poses_.poses.push_back(currentpose_);
        RCLCPP_INFO(node_->get_logger(), "Adding pose for the first time");
        return true;
    }

    for (const auto& pose_rcv : received_poses_.poses) 
    {
        RCLCPP_INFO(node_->get_logger(), "Validating if pose is within (%f,%f) : %ld total", 
            pose_rcv.position.x,
            pose_rcv.position.y,
            received_poses_.poses.size());

        // Calculate the limits for the perimeter
        double x_lower_limit = pose_rcv.position.x - spray_rad_m;
        double x_upper_limit = pose_rcv.position.x + spray_rad_m;
        double y_lower_limit = pose_rcv.position.y - spray_rad_m;
        double y_upper_limit = pose_rcv.position.y + spray_rad_m;
        RCLCPP_INFO(node_->get_logger(), "limits: x(%f,%f) :y(%f,%f)", 
            x_lower_limit, x_upper_limit, y_lower_limit, y_upper_limit
        );

        if(currentpose_.position.x >= x_lower_limit &&
            currentpose_.position.x <= x_upper_limit &&
            currentpose_.position.y >= y_lower_limit &&
            currentpose_.position.y <= y_upper_limit)
        {
            RCLCPP_INFO(node_->get_logger(), "Pose already exist, skip disinfection!");
            return false;
        }
    }
    received_poses_.poses.push_back(currentpose_);
    RCLCPP_INFO(node_->get_logger(), "New Pose detected..add");
    return true;
}