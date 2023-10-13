#pragma once

#include <geometry_msgs/msg/pose.hpp> 

class SharedObjPose
{
public:
    static SharedObjPose& getInstance()
    {
        static SharedObjPose instance;
        return instance;
    }

    geometry_msgs::msg::Pose getPose() const { return pose_; }
    void setPose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }

private:
    geometry_msgs::msg::Pose pose_;
    SharedObjPose() {}
};