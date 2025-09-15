cmake_minimum_required(VERSION 3.8)
project(rpi5_gpio_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(std_msgs REQUIRED)


# Hardware (forwarder)
add_library(rpi5_forward_system SHARED src/rpi5_forward_system.cpp)
target_include_directories(rpi5_forward_system PUBLIC include)
ament_target_dependencies(rpi5_forward_system
  hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)

# Controller
add_library(rpi5_valve_controller SHARED src/rpi5_valve_controller.cpp)
target_include_directories(rpi5_valve_controller PUBLIC include)
ament_target_dependencies(rpi5_valve_controller
  controller_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)

# Export plugin descriptions
pluginlib_export_plugin_description_file(hardware_interface rpi5_gpio_control.xml)
pluginlib_export_plugin_description_file(controller_interface rpi5_gpio_control.xml)

install(TARGETS rpi5_forward_system rpi5_valve_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include DESTINATION include)
install(FILES rpi5_gpio_control.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
