#include "turtlefied/save_map.hpp"
#include <iomanip>
#include <ctime>
#include <sstream>
#include "turtlefied/shared_mapstatus.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace BT;

SaveMapBT::SaveMapBT(const std::string& name, const NodeConfiguration& conf)
    : SyncActionNode(name, conf)
{
    auto er = getInput("map_name", map_name_);
    if(map_name_ == "")
    {
        throw RuntimeError("error reading port [map_name]:", er.error());
    }
    map_node_ = rclcpp::Node::make_shared("bt_map_saver");
    map_client_ = map_node_->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
    is_map_saved = false;
}

NodeStatus SaveMapBT::tick(){
    if(is_map_saved){
        std::cout << "Map is already Saved" << std::endl;
        return NodeStatus::FAILURE;
    }
    auto request_ = std::make_shared<nav2_msgs::srv::SaveMap::Request>();

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d%H%M%S");
    auto str = oss.str();

    std::string file_name = map_name_ + str;
    std::cout << "Map will Save as : "<< file_name << std::endl;

    request_->map_topic = "map";
    request_->map_url = file_name;
    request_->image_format = "pgm";
    request_->map_mode = "trinary";
    request_->free_thresh = 0.25;
    request_->occupied_thresh = 0.65;
    
    while(!map_client_->wait_for_service(5s)){
        if(!rclcpp::ok()){
            return BT::NodeStatus::FAILURE;
        }
        std::cout <<"Map service not available...waiting..." << std::endl;
    }
    auto future_result_ = map_client_->async_send_request(request_);
    if (rclcpp::spin_until_future_complete(map_node_, future_result_) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "Map Saved Successfully" <<std::endl;
        is_map_saved = true;
        SharedMapStatus::getInstance().setMapStatus(is_map_saved);
        setOutput("set_map_dir", file_name);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << "FAIL TO CALL SERVICE!!!!!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

IsMapSaved::IsMapSaved(const std::string& name, const NodeConfiguration& conf)
    : ConditionNode(name, conf)
{
    map_node_ = rclcpp::Node::make_shared("bt_map_status");
    is_map_already_saved = false;
}
IsMapSaved::~IsMapSaved()
{
    cleanup();
}

NodeStatus IsMapSaved::tick()
{
    is_map_already_saved = SharedMapStatus::getInstance().getMapStatus();
    if(is_map_already_saved)
    {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}