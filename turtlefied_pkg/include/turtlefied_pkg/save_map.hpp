#ifndef TURTLEFIED_PKG__SAVE_MAP_HPP__
#define TURTLEFIED_PKG__SAVE_MAP_HPP__

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/save_map.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class SaveMapBT : public SyncActionNode
{
public:
    SaveMapBT(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts(){
        return {
            InputPort<std::string>("map_name", "The name of the Map"),
            OutputPort<std::string>("set_map_dir", "Store Map Directory")
        };
    }
    virtual NodeStatus tick() override;
private:
    rclcpp::Node::SharedPtr map_node_;
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_client_;
    std::string map_name_;
    bool is_map_saved;
};

class IsMapSaved : public ConditionNode
{
public:
    IsMapSaved(const std::string& name, const NodeConfiguration& conf);
    static PortsList providedPorts(){return {};}
    NodeStatus tick() override;
    IsMapSaved() = delete;
    ~IsMapSaved() override;

protected:
    void cleanup(){}

private:
    rclcpp::Node::SharedPtr map_node_;
    bool is_map_already_saved;
};

#endif //TURTLEFIED_PKG__SAVE_MAP_HPP__