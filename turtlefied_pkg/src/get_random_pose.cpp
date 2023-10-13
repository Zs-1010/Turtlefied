#include "turtlefied_pkg//get_random_pose.hpp"
#include "turtlefied_pkg//shared_pose.hpp"


using namespace BT;
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
using namespace std::chrono_literals;

GoToRandomPose::GoToRandomPose(const std::string& name, const NodeConfiguration& conf)
    :   StatefulActionNode(name, conf)
{
    node_ = rclcpp::Node::make_shared("get_random_pose");
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap",
            10,
            std::bind(&GoToRandomPose::getrandposecallback, this, std::placeholders::_1)
        );
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10
        );

    nav_sub_ = node_->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
            "/navigate_to_pose/_action/feedback",
            10,
            std::bind(&GoToRandomPose::checkrandposestatcallback, this, std::placeholders::_1)
        );
    dist_ = 10.0;
}

void GoToRandomPose::getrandposecallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg_)
{
    std::vector<int8_t> mapData = msg_->data;
    int mapWidth = msg_->info.width;
    int mapHeight = msg_->info.height;
    double mapRes = msg_->info.resolution;
    geometry_msgs::msg::Pose mapOrigin = msg_->info.origin;

    for(int y=0; y<mapHeight; ++y){
        for(int x=0; x<mapWidth; ++x) {
            int cell_index = y * mapWidth + x;
            if(mapData[cell_index] == 0){
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = msg_->header.frame_id;
                pose.header.stamp = node_->get_clock()->now();
                pose.pose.position.x = mapOrigin.position.x + (x+0.5) * mapRes;
                pose.pose.position.y = mapOrigin.position.y + (y+0.5) * mapRes;
                pose.pose.position.z = 0.0; //2D Map

                available_poses.push_back(pose);
            }
        }
    }

}
void GoToRandomPose::checkrandposestatcallback(const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr stat_)
{
    dist_ = stat_->feedback.distance_remaining;
}

NodeStatus GoToRandomPose::onStart()
{
    int msec = 30000; //30sec
    deadline_ = system_clock::now() + milliseconds(msec);
    rclcpp::spin_some(node_);
    if(!available_poses.empty()){
        int random_index = std::rand() % available_poses.size();
        pub_->publish(available_poses[random_index]);
        RCLCPP_INFO(node_->get_logger(), "Published a random goal pose: (%.2f, %.2f)",
            available_poses[random_index].pose.position.x, available_poses[random_index].pose.position.y);
        return NodeStatus::RUNNING;
    }
    return NodeStatus::FAILURE;
}

NodeStatus GoToRandomPose::onRunning()
{
    if(system_clock::now() >= deadline_){
        RCLCPP_INFO(node_->get_logger(), "Timeout, trying new goal pose");
        return NodeStatus::FAILURE;
    }
    rclcpp::spin_some(node_);
    if(dist_ <=0.3)
    {
        RCLCPP_INFO(node_->get_logger(), "Distance Reached!");
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

void GoToRandomPose::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Interrupted!");
}


EnableObjectDetection::EnableObjectDetection(const std::string& name, const NodeConfiguration& conf)
    :   SyncActionNode(name, conf)
{
    node_ = rclcpp::Node::make_shared("search_object");
    ope_client = node_->create_client<modpi_custom_msgs::srv::ObjCoor>("/get_object_coor");
    node_->get_parameter("use_sim_time").get_parameter_value().get<bool>();
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("use_sim_time", true)};
    node_->set_parameters(all_new_parameters);
}

NodeStatus EnableObjectDetection::tick()
{
    RCLCPP_INFO(node_->get_logger(), "Searching Object");
    auto request = std::make_shared<modpi_custom_msgs::srv::ObjCoor::Request>();
    while(!ope_client->wait_for_service(5s)){
        if(!rclcpp::ok()){
            return NodeStatus::FAILURE;
        }
        std::cout << "service not available, waiting again...!!!!!" << std::endl;
    }
    auto future_result = ope_client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node_,future_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        {
            if(!future_result.get()->success){
                RCLCPP_INFO(node_->get_logger(), "No Object");
                return NodeStatus::FAILURE;
            }
            else{
                RCLCPP_INFO(node_->get_logger(), "Got Message- %s : d=%.2f",
                             future_result.get()->classname.c_str(),
                             future_result.get()->distance);

                //Compute to pose
                geometry_msgs::msg::Pose op_;
                float cx = 320.0f;      //324.542724609375
                float cy = 180.0f;      //240.71603393554688
                double fx = 320.2549;   //384.41644287109375
                double fy = 320.2549;   //384.41644287109375
                double x_center = (future_result.get()->xmax - future_result.get()->xmin)/2 + future_result.get()->xmin;
                double y_center = (future_result.get()->ymax - future_result.get()->ymin)/2 + future_result.get()->ymin;
                
                op_.position.x = (x_center-cx)*future_result.get()->distance/fx;
                op_.position.y = (y_center-cy)*future_result.get()->distance/fy;
                op_.position.z = future_result.get()->distance;

                SharedPose::getInstance().setPose(op_);
                RCLCPP_INFO(node_->get_logger(), "Got Object Pose: (%.2f,%.2f,%.2f) relative to camera",
                            op_.position.x,
                            op_.position.y,
                            op_.position.z);
                return NodeStatus::SUCCESS;
            }
        }
    else{
        RCLCPP_INFO(node_->get_logger(), "Failed to call service");
        return NodeStatus::FAILURE;
    }
}

RotateInPlace::RotateInPlace(const std::string& name, const NodeConfiguration& conf)
    :   SyncActionNode(name, conf)
{
    node_ = rclcpp::Node::make_shared("rotate_in_place");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );
    getInput("direction", direction_);
    if (direction_ == "clockwise"){
        speed_ = -0.30;
    }
    else if(direction_ == "counterclockwise"){
        speed_ = 0.30;
    }
    else{RCLCPP_INFO(node_->get_logger(), "default direction is counterclockwise!");  }
}

NodeStatus RotateInPlace::tick()
{
    geometry_msgs::msg::Twist rotate;
    int count_ = 0;
    while(count_ < 3)
    {
        rotate.angular.z = speed_;
        pub_->publish(rotate);
        count_++;
        RCLCPP_INFO(node_->get_logger(), "rotate");   
        SleepMS(1000);
    }
    
    rotate.angular.z = 0.0;
    pub_->publish(rotate);
    RCLCPP_INFO(node_->get_logger(), "rotate Stopped!");  

    return NodeStatus::SUCCESS;
}