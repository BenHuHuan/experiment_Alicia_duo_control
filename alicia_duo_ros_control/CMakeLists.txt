cmake_minimum_required(VERSION 3.5)
project(alicia_duo_ros_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(alicia_duo_ros_control SHARED
  src/alicia_hardware_interface.cpp
)

target_include_directories(alicia_duo_ros_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(alicia_duo_ros_control
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

# ✅ 明确导出和安装的库名
install(TARGETS alicia_duo_ros_control
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(alicia_duo_ros_control)

ament_package()

