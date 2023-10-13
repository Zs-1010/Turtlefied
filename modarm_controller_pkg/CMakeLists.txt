cmake_minimum_required(VERSION 3.8)
project(modarm_controller_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(modpi_custom_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_executable(arm_control src/arm_control_hw.cpp)
ament_target_dependencies(arm_control rclcpp trajectory_msgs sensor_msgs)

add_executable(move_to_pose src/move_to_pose.cpp)
target_link_libraries(move_to_pose ${rclcpp_LIBRARIES})
ament_target_dependencies(move_to_pose rclcpp modpi_custom_msgs sensor_msgs)

add_executable(test_trigger src/test_trigger.cpp)
target_link_libraries(test_trigger ${rclcpp_LIBRARIES})
ament_target_dependencies(test_trigger rclcpp example_interfaces sensor_msgs)

install(TARGETS
arm_control
move_to_pose
test_trigger
  DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED 
  src/servo_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}
 "${SERIAL_LDFLAGS}")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface servo_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  DESTINATION include
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
