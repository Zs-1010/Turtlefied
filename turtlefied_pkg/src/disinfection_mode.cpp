#include "turtlefied_pkg//disinfection_mode.hpp"
#include <set>

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

using namespace BT;

DisinfectionMode::DisinfectionMode(const std::string& name, 
    const NodeConfiguration& conf)
:   SyncActionNode(name, conf)
{
    d_node_ = rclcpp::Node::make_shared("trigger_spray_uvc");
    getInput("pin_nbr", gpio_pin_);
    getInput("timer_s", timer_s_);

    std::set<int> allowedPins = {23, 24, 25, 26, 16,
                     20, 21, 17, 27, 22, 5, 6,19,26};
    if(allowedPins.count(gpio_pin_) == 0){
        RCLCPP_ERROR(d_node_->get_logger(), 
        "Pin %d is not allowed, default pins will be set to [23] timer=%d seconds",
        gpio_pin_, timer_s_);
        gpio_pin_=23;
    }
    RCLCPP_INFO(d_node_->get_logger(), 
        "Pin set %d timer=%d seconds",
        gpio_pin_, timer_s_);

    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        RCLCPP_ERROR(d_node_->get_logger(),"Failed to open GPIO chip");
    }
    gpio_ = gpiod_chip_get_line(chip, gpio_pin_);

    if (!gpio_) {
        RCLCPP_ERROR(d_node_->get_logger(), "Failed to get GPIO lines");
        gpiod_chip_close(chip);
    }

    if (gpiod_line_request_output(gpio_, "test-libgpiod-1", 0) < 0)
    {
        RCLCPP_ERROR(d_node_->get_logger(),"Failed to request GPIO lines as outputs");
        gpiod_chip_close(chip);
    }
    gpiod_line_request_output(gpio_, "test-libgpiod-1", 0);

}

DisinfectionMode::~DisinfectionMode()
{
    if (chip) 
    {
        RCLCPP_INFO(d_node_->get_logger(),"Cleanup");
        gpiod_chip_close(chip);
    }
    cleanup();
}
NodeStatus DisinfectionMode::tick()
{
    for (int i = 0; i < timer_s_; ++i) 
    {
        gpiod_line_set_value(gpio_, 1);
        SleepMS(1000);
    }

    gpiod_line_set_value(gpio_, 0);
    return NodeStatus::SUCCESS;
}