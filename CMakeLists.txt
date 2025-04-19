cmake_minimum_required(VERSION 3.8)
project(jaeger_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


find_package(ignition-cmake2 REQUIRED)

# Add Gazebo dependencies
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

add_library(${PROJECT_NAME} SHARED src/hello_world.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME} 
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
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