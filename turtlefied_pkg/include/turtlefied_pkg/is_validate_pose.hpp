#ifndef TURTLEFIED_PKG__IS_VALIDATE_POSE_HPP__
#define TURTLEFIED_PKG__IS_VALIDATE_POSE_HPP__

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"

using namespace std::chrono;
using namespace BT;

class IsPoseExists : public ConditionNode
{
public:
    IsPoseExists(const std::string& condition_name, const NodeConfiguration& conf);
    NodeStatus tick() override;
    IsPoseExists() = delete;
    ~IsPoseExists() override;
    void initialize();
    bool IsPoseThere();
    static PortsList providedPorts(){
        return {
            InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
            InputPort<std::string>("source_frame", std::string("base_footprint"), "Robot base frame"),
            InputPort<double>("coverage", 0.5, "Reach of Spray in meter(s)"),
            InputPort<int>("timeout",10000, "Timeout to execute Transform in ms")
        };
    }

protected:
    void cleanup(){}

private:
    rclcpp::Node::SharedPtr node_;
    geometry_msgs::msg::Pose currentpose_;
    geometry_msgs::msg::PoseArray received_poses_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool initialized_;
    std::string global_frame_;
    std::string robot_base_frame_;
    double spray_rad_m;
    system_clock::time_point deadline_;
    int msec;
};


#endif //TURTLEFIED_PKG__IS_VALIDATE_POSE_HPP__
