// shared_data_storage.hpp

#pragma once

#include <geometry_msgs/msg/pose2_d.hpp> 

class SharedDataStorage
{
public:
    static SharedDataStorage& getInstance()
    {
        static SharedDataStorage instance;
        return instance;
    }

    geometry_msgs::msg::Pose2D getOriginPose() const { return origin_pose_; }
    void setOriginPose(const geometry_msgs::msg::Pose2D& pose) { origin_pose_ = pose; }

private:
    geometry_msgs::msg::Pose2D origin_pose_;
    SharedDataStorage() {}  // Private constructor to enforce singleton
};