cmake_minimum_required(VERSION 3.16)
project(piper_hardware_interface VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(realtime_tools REQUIRED)
find_package(libpiper 0.5 CONFIG REQUIRED)
set(PIPER_LIBPIPER_TARGET libpiper::libpiper)

add_library(piper_hardware_interface SHARED
  src/piper_hardware_interface.cpp
  src/piper_gripper_interface.cpp
)

target_include_directories(piper_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(piper_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# Extend link libraries with plain syntax to match ament_target_dependencies (no keyword mix).
target_link_libraries(piper_hardware_interface ${PIPER_LIBPIPER_TARGET} Eigen3::Eigen)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

install(
  TARGETS piper_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
  libpiper
  realtime_tools
)
ament_package()
