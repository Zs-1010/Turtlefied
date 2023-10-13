#pragma once

#include <geometry_msgs/msg/pose.hpp> 

class SharedPose
{
public:
    static SharedPose& getInstance()
    {
        static SharedPose instance;
        return instance;
    }

    geometry_msgs::msg::Pose getPose() const { return pose_; }
    void setPose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }

private:
    geometry_msgs::msg::Pose pose_;
    SharedPose() {}
};