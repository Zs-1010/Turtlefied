#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "modpi_ik_pkg/srv/set_arm_pose.hpp"

using namespace std::placeholders;

class MoveitPoseServiceServer : public rclcpp::Node
{
public:
  MoveitPoseServiceServer() : Node("move_arm")
  {
    service_ = create_service<modpi_ik_pkg::srv::SetArmPose>("set_arm_pose",
          std::bind(&MoveitPoseServiceServer::servicecallback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger("moveit2"), "Service Ready");
  }
  void send_default_rqst_()
  {
    auto client_ = this->create_client<modpi_ik_pkg::srv::SetArmPose>("set_arm_pose");
    RCLCPP_INFO(rclcpp::get_logger("moveit2"), "preparing the arm");
    // Create a request and send it to the service
    auto request = std::make_shared<modpi_ik_pkg::srv::SetArmPose::Request>();
    request->target_name = "ObjectDetectionState";
    auto future = client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    // Process the response (if needed)
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto response = future.get();
      RCLCPP_INFO(rclcpp::get_logger("moveit2"), "DONE!@@@@");
    }
  }
private:
  rclcpp::Service<modpi_ik_pkg::srv::SetArmPose>::SharedPtr service_;
  void servicecallback(const std::shared_ptr<modpi_ik_pkg::srv::SetArmPose::Request> request_,
                        const std::shared_ptr<modpi_ik_pkg::srv::SetArmPose::Response> response_
                      )
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("moveit2"), "Request Pose Received " << request_->target_name);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("moveit2"), "success = " << response_->isreturnsuccess);
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(this->shared_from_this(), "arm_planning_group");
    move_group_interface.setNamedTarget(request_->target_name);
    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
    move_group_interface.execute(plan);
    } else {
    RCLCPP_ERROR(rclcpp::get_logger("moveit2"), "Planning failed!");
    }

    response_->isreturnsuccess = true;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("moveit2"), "success = " << response_->isreturnsuccess);
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveitPoseServiceServer>();
  node->send_default_rqst_();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}