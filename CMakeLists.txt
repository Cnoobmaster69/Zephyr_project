cmake_minimum_required(VERSION 3.8)
project(jaeger_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Dependencias para ros2_control
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(controller_manager REQUIRED)




# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()


find_package(gz-cmake3 REQUIRED)

# Add Gazebo dependencies
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Gazebo Sim
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(${PROJECT_NAME} SHARED src/hello_world.cc 
  hardware/my_robot_hardware.cpp
  controller/my_robot_controller.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

target_include_directories(jaeger_model PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/jaeger_model>
)

target_link_libraries(${PROJECT_NAME}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  controller_manager::controller_manager
  rcl_lifecycle::rcl_lifecycle
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  
  )

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_plugin.xml)

pluginlib_export_plugin_description_file(controller_interface my_robot_hardware_plugin.xml)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  nav_msgs
  rcl_lifecycle
  controller_manager
  rclcpp_lifecycle
  realtime_tools
  hardware_interface
  controller_interface
  pluginlib
  tf2
  tf2_geometry_msgs
)


install(
  DIRECTORY hardware/include/
  DESTINATION include/jaeger_model
)
install(
  DIRECTORY controller/include/
  DESTINATION include/jaeger_model
)

install(
  DIRECTORY
    config
    launch
    urdf
    worlds
    meshes
    src
  DESTINATION
    share/${PROJECT_NAME}/
)
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib)


ament_package()