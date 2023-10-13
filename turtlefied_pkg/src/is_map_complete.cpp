#include "turtlefied/is_map_complete.hpp"

using namespace BT;

IsMapComplete::IsMapComplete(
    const std::string& condition_name,
    const NodeConfiguration& conf)
: ConditionNode(condition_name, conf),
    is_map_reached(false),
    percent_map_(0),
    free(0),
    obstacle(0),
    gridSize(0),
    free_progress_(0.0),
    obs_progress_(0.0)
{
    getInput("map_target", percent_map_);
    node_ = rclcpp::Node::make_shared("is_map_complete");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
    callback_group_exec_.add_callback_group(callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    grid_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&IsMapComplete::map_callback, this, std::placeholders::_1),
        sub_option);
}

NodeStatus IsMapComplete::tick()
{
    callback_group_exec_.spin_some();
    if(is_map_reached){
        RCLCPP_INFO(node_->get_logger(), "Map is reached");
        is_map_reached = false;
        free = 0;
        obstacle = 0;
        gridSize = 0;
        free_progress_ = 0.0;
        obs_progress_ = 0.0;
        return NodeStatus::SUCCESS;
    }
    free = 0;
    obstacle = 0;
    gridSize = 0;
    free_progress_ = 0.0;
    obs_progress_ = 0.0;
    return NodeStatus::FAILURE;
}

void IsMapComplete::map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr grid_info)
{
    for(int i: grid_info->data){
        if(i>=100){
            obstacle++;
        }
        else if(i == 0){
            free++;
        }
    }
    gridSize = grid_info->data.size();
    free_progress_ = static_cast<double>(free)/gridSize;
    obs_progress_ = static_cast<double>(obstacle)/gridSize;
    is_map_reached = (free_progress_+obs_progress_>=percent_map_);
    RCLCPP_INFO(node_->get_logger(), "Map Status: %.2f/%.2f Percent", (free_progress_+obs_progress_)*100.0, percent_map_*100.0);
}