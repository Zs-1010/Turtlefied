#include "rclcpp/rclcpp.hpp"

#include "turtlefied_pkg//explore_client.hpp"

/**
 * @brief
 */
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
using namespace std::chrono;
using namespace BT;

FetchGoal::FetchGoal(const std::string& name, const NodeConfiguration& config) 
        : StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("find_closest_frontier");
        subscr_ = node_->create_subscription<geometry_msgs::msg::Point>(
            "/explore/goal",
            10,
            std::bind(&FetchGoal::callback, this, std::placeholders::_1)
        );

        pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10
        );
        point_received_ = false;
    }

//mandator to define this STATIC Method
PortsList FetchGoal::providedPorts(){return {};}

void FetchGoal::callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // Callback logic when a new message arrives
    received_point_ = *msg;
    RCLCPP_INFO(node_->get_logger(), "Received Available Frontiers!!");
    point_received_ = true;
}

//Override the virtual fx onStart()
NodeStatus FetchGoal::onStart()
{
    int msec = 30000;
    deadline_ = system_clock::now() + milliseconds(msec);

    RCLCPP_INFO(node_->get_logger(), "Find Available Frontiers!");
    return NodeStatus::RUNNING;
}

//Override the virtual fx onRunning()
NodeStatus FetchGoal::onRunning()
{
    if(system_clock::now() >= deadline_){
        RCLCPP_INFO(node_->get_logger(), "timeout!! Publish Last Pose");
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = received_point_.x;
        pose_msg.pose.position.y = received_point_.y;
        pose_msg.pose.position.z = received_point_.z;
        pub_->publish(pose_msg);
        return NodeStatus::SUCCESS;
        // halt();
    }
    else if (point_received_) {
        RCLCPP_INFO(node_->get_logger(), "Received goal: x=%.2f, y=%.2f, z=%.2f",
                    received_point_.x, received_point_.y, received_point_.z);
        // Create a Pose message and publish it on topic
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = received_point_.x;
        pose_msg.pose.position.y = received_point_.y;
        pose_msg.pose.position.z = received_point_.z;
        pub_->publish(pose_msg);

        point_received_ = false;
        return NodeStatus::SUCCESS;
    }

    rclcpp::spin_some(node_);
    SleepMS(1000);
    return NodeStatus::RUNNING;
}

void FetchGoal::onHalted()
{
    std::cout << "FetchGoal interrupted!" << std::endl;
}