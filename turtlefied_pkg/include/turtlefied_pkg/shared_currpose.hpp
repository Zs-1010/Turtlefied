// shared_currpose.hpp

#pragma once

#include <geometry_msgs/msg/pose.hpp>

class SharedCurrPose
{
public:
    static SharedCurrPose& getInstance()
    {
        static SharedCurrPose instance;
        return instance;
    }

    geometry_msgs::msg::Pose getPose() const { return curr_pose_; }
    void setPose(const geometry_msgs::msg::Pose& currpose) { curr_pose_ = currpose; }

private:
    geometry_msgs::msg::Pose curr_pose_;
    SharedCurrPose() {}  // Private constructor to enforce singleton
};