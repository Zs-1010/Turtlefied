#pragma once

#include <geometry_msgs/msg/pose.hpp> 

class SharedRejectedPose
{
public:
    static SharedRejectedPose& getInstance()
    {
        static SharedRejectedPose instance;
        return instance;
    }

    geometry_msgs::msg::Pose getrejectedPose() const { return rejected_pose_; }
    void setrejectedPose(const geometry_msgs::msg::Pose& pose) { rejected_pose_ = pose; }
    void clear() { rejected_pose_ = geometry_msgs::msg::Pose(); }

private:
    geometry_msgs::msg::Pose rejected_pose_;
    SharedRejectedPose() {}
};