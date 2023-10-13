#include "modarm_controller_pkg/servo_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace servo_controller
{
ServoInterface::ServoInterface(){}
ServoInterface::~ServoInterface()
{
    if(servo_.IsOpen())
    {
        try
        {
            servo_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ServoInterface"),
                    "Oh oh something went wrong while closing the port!" << port_);
        }
    }
}

CallbackReturn ServoInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    //Initializing Ports
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS)
    {
        return result;
    }
    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("ServoInterface"), "No serial port has been provided");
        return CallbackReturn::FAILURE;
    }

    position_cmd_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    last_position_cmd_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> ServoInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    //collect command_interfaces
    for(size_t i=0; i<info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_cmd_[i]));
    }

    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> ServoInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    //collect state_interfaces
    for(size_t i=0; i<info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }
    return state_interfaces;
}

CallbackReturn ServoInterface::on_activate(const rclcpp_lifecycle::State& last_state)
{
    RCLCPP_INFO(rclcpp::get_logger("ServoInterface"), "Initialize Robot Arm...");

    //reset position's command and state
    position_cmd_ = {0.0, 0.0, 0.0, 0.0};
    last_position_cmd_ = {0.0, 0.0, 0.0, 0.0};
    position_states_ = {0.0, 0.0, 0.0, 0.0};
    try{
        servo_.Open(port_);
        servo_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("ServoInterface"), 
                "something went wrong opening port " << port_);
    }

    RCLCPP_INFO(rclcpp::get_logger("ServoInterface"), "hardware is ready to take commands!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoInterface::on_deactivate(const rclcpp_lifecycle::State& last_state)
{
    if(servo_.IsOpen())
    try{
        servo_.Close();
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("ServoInterface"),
            "something went wrong closing port " << port_);
    }

    RCLCPP_INFO(rclcpp::get_logger("ServoInterface"), "Hardware Stopped: Closed the port successfully");
    return CallbackReturn::SUCCESS;
}
 
hardware_interface::return_type ServoInterface::read()                                                                                                     
{
    position_states_ = position_cmd_;
    return hardware_interface::return_type::OK;
}
    
hardware_interface::return_type ServoInterface::write()                                                                                                    
{
    if(position_cmd_ == last_position_cmd_)
    {
        return hardware_interface::return_type::OK;
    }
    //send commands
    std::string msg;
    int base = static_cast<int>((position_cmd_.at(0) *180)/M_PI);
    msg.append(std::to_string(base));
    msg.append(",");
    int shoulder = static_cast<int>((position_cmd_.at(1) *180) /M_PI);
    msg.append(std::to_string(shoulder));
    msg.append(",");
    int elbow = static_cast<int>((position_cmd_.at(2) *180) /M_PI);
    msg.append(std::to_string(elbow));
    msg.append(",");
    int wrist = static_cast<int>((position_cmd_.at(3) *180) /M_PI);
    msg.append(std::to_string(wrist));
    msg.append(",.");
    
    try{
        RCLCPP_INFO_STREAM(rclcpp::get_logger("ServoInterface"), "Sending new position commands " << msg);
        servo_.Write(msg);
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("ServoInterface"), "Cannot Send "<<msg <<" to port " << port_);
        return hardware_interface::return_type::ERROR;
    }

    last_position_cmd_ = position_cmd_;
    return hardware_interface::return_type::OK;

}
}   //servo_controller namespace
PLUGINLIB_EXPORT_CLASS(servo_controller::ServoInterface, hardware_interface::SystemInterface)