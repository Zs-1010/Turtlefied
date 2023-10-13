#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "turtlefied_pkg/explore_client.hpp"
#include "turtlefied_pkg/check_if_succeed.hpp"
#include "turtlefied_pkg/return_to_orig.hpp"
#include "turtlefied_pkg/is_distance_reached.hpp"
#include "turtlefied_pkg/robot_pause.hpp"
#include "turtlefied_pkg/robot_resume.hpp"
#include "turtlefied_pkg/is_map_complete.hpp"
#include "turtlefied_pkg/is_return_to_origin.hpp"
#include "turtlefied_pkg/save_map.hpp"
#include "turtlefied_pkg/is_validate_pose.hpp"
#include "turtlefied_pkg/disinfection_mode.hpp"
#include "turtlefied_pkg/transition_to_.hpp"
#include "turtlefied_pkg/move_arm.hpp"
#include "turtlefied_pkg/get_random_pose.hpp"
#include "turtlefied_pkg/compute_obj_to_pose.hpp"
#include "turtlefied_pkg/navigate_to_obj.hpp"
#include "turtlefied_pkg/obj_broadcaster.hpp"

using namespace BT;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  std::filesystem::path ros_ws_path = std::filesystem::current_path();
  std::string xml_file_name = "/turtlefied_pkg/config/t_bt_ros2.xml";
  std::string xml_path = ros_ws_path.string() + xml_file_name;

  rclcpp::init(argc, argv);    
  BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(xml_path);

  int msec = 900000; //runtime
  system_clock::time_point time_limit = system_clock::now() + milliseconds(msec);
  try{
    while(system_clock::now() <= time_limit)
    {
      tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Delay between ticks
    }
    std::cout << "Runtime is set for t-mins only BT will exit" << std::endl;
  } catch (const std::exception& e) {
        std::cout << "Exception:" << e.what() << std::endl;
  } catch (...) {
      std::cout << "Unknown exception occurred" << std::endl;
  }


  // let's visualize some information about the current state of the blackboards.
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[0]->debugMessage();
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[1]->debugMessage();
  std::cout << "--------------" << std::endl;

  return 0;
}