#pragma once
#ifndef SERVO_INTERFACE_H
#define SERVO_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>

namespace servo_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ServoInterface : public hardware_interface::SystemInterface
{
public:
    ServoInterface();
    virtual ~ServoInterface();

    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &last_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &last_state) override;
    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual hardware_interface::return_type read() override;
    virtual hardware_interface::return_type write() override;


private:
    LibSerial::SerialPort servo_;
    std::string port_;
    std::vector<double> position_cmd_;
    std::vector<double> last_position_cmd_;
    std::vector<double> position_states_;
};

} //namespace servo_controller
#endif 