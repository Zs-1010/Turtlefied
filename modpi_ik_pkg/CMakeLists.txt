cmake_minimum_required(VERSION 3.8)
project(modpi_ik_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning)

# custom msgs stuff
find_package(std_msgs REQUIRED)
find_package(modpi_ik_pkg REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetArmPose.srv"
)
# Install the service message header files
install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# Install Cpp executables
add_executable(move_arm src/arm_ik.cpp)
target_link_libraries(move_arm ${rclcpp_LIBRARIES})
ament_target_dependencies(move_arm rclcpp moveit_msgs moveit_ros_planning_interface std_msgs modpi_ik_pkg)

# iNSTALL
install(TARGETS 
  move_arm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()




