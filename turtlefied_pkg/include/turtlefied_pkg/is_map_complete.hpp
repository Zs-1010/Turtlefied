#ifndef TURTLEFIED_PKG__IS_MAP_COMPLETE_HPP
#define TURTLEFIED_PKG__IS_MAP_COMPLETE_HPP

#include <string>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
class IsMapComplete : public ConditionNode
{
public:
    IsMapComplete(const std::string& condition_name, const NodeConfiguration& conf);
    virtual NodeStatus tick() override;
    IsMapComplete() = delete;
    static PortsList providedPorts(){
        return  {
            InputPort<std::string>("map_target", "Map progress in percentage"),
        };
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_exec_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;

    void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr grid_info);
    bool is_map_reached;
    double percent_map_;

    size_t free;
    size_t obstacle;
    size_t gridSize;
    double free_progress_;
    double obs_progress_ ;
};

#endif //TURTLEFIED_PKG__IS_MAP_COMPLETE_HPP