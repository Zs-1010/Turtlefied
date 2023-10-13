cmake_minimum_required(VERSION 3.8)
project(turtlefied_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(modpi_ik_pkg REQUIRED)
find_package(modpi_custom_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  tf2_ros
  modpi_ik_pkg
  modpi_custom_msgs
)  

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)
find_library(LIBGPIOD_LIBRARY NAMES gpiod PATHS /usr/lib/x86_64-linux-gnu)

add_executable(bt_decision src/bt_decision.cpp src/explore_client.cpp src/check_if_succeed.cpp
        src/return_to_orig.cpp src/is_distance_reached.cpp src/robot_pause.cpp
        src/robot_resume.cpp src/is_map_complete.cpp src/is_return_to_origin.cpp
        src/save_map.cpp src/is_validate_pose.cpp src/disinfection_mode.cpp 
        src/transition_to_.cpp src/move_arm.cpp
        src/get_random_pose.cpp src/compute_obj_to_pose.cpp
        src/navigate_to_obj.cpp src/obj_broadcaster.cpp)
ament_target_dependencies(bt_decision ${dependencies})
target_link_libraries(bt_decision ${LIBGPIOD_LIBRARY})

install(TARGETS
  bt_decision
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch meshes urdf gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
