#ifndef TURTLEFIED_PKG__GET_OBJECT_POSE_HPP
#define TURTLEFIED_PKG__GET_OBJECT_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
//#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"
    /*  This is a custom message
            std_msgs/Header header
            std_msgs/Header image_header
            BoundingBox[] bounding_boxes
                bounding_box.msg 
                float32 probability
                uint16 xmin
                uint16 ymin
                uint16 xmax
                uint16 ymax
                uint16 id
                uint16 img_width
                uint16 img_height
                int32 center_dist
    */

#include <geometry_msgs/msg/pose_stamped.hpp>


#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class GetObjectPose : public SyncActionNode
{
public:
    GetObjectPose(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts();
    virtual NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr obj_client_;
    rclcpp::Subscriber<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
    rclcpp::Subscriber<bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr bb_sub_;
    rclcpp::Service<geometry_msgs::msg::PoseStamped>::SharedPtr obj_service_;
}

#endif  //TURTLEFIED_PKG__GET_OBJECT_POSE_HPP