#ifndef TURTLEFIED_PKG__DISINFECTION_MODE_HPP__
#define TURTLEFIED_PKG__DISINFECTION_MODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <gpiod.h>
#include <unistd.h>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class DisinfectionMode : public SyncActionNode
{
public:
    DisinfectionMode(const std::string& name, const NodeConfiguration& conf);
    DisinfectionMode() = delete;
    ~DisinfectionMode() override;
    static PortsList providedPorts(){
        return {
            InputPort<int>("pin_nbr", 23, "assign GPIO pin"),
            InputPort<int>("timer_s", 10, "Disinfection timer in seconds")
        };
    }
    virtual NodeStatus tick() override;
protected:
    void cleanup(){}
private:
    rclcpp::Node::SharedPtr d_node_;
    int gpio_pin_;
    int timer_s_;
    struct gpiod_chip *chip;
    struct gpiod_line *gpio_;
};

#endif //TURTLEFIED_PKG__DISINFECTION_MODE_HPP__
