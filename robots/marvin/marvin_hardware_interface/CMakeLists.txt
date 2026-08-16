cmake_minimum_required(VERSION 3.16)
project(marvin_hardware_interface VERSION 0.1.0 LANGUAGES CXX)

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
find_package(libmarvin CONFIG REQUIRED)
set(MARVIN_LIBMARVIN_TARGET libmarvin::libmarvin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED
  src/marvin_sdk_bridge.cpp
  src/marvin_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Marvin SDK headers need CMPL_LIN for Linux platform types.
target_compile_definitions(${PROJECT_NAME} PRIVATE CMPL_LIN)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} ${MARVIN_LIBMARVIN_TARGET})

pluginlib_export_plugin_description_file(hardware_interface marvin_hardware_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  libmarvin
)
# --- Tests ---

if(BUILD_TESTING)
  ament_add_gtest(marvin_sdk_bridge_test
    test/marvin_sdk_bridge_test.cpp
    src/marvin_sdk_bridge.cpp
  )
  target_include_directories(marvin_sdk_bridge_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_definitions(marvin_sdk_bridge_test PRIVATE CMPL_LIN)

  ament_add_gtest(marvin_write_guard_test
    test/marvin_write_guard_test.cpp
    src/marvin_sdk_bridge.cpp
  )
  target_include_directories(marvin_write_guard_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_definitions(marvin_write_guard_test PRIVATE CMPL_LIN)

  ament_add_gtest(marvin_sdk_transaction_test
    test/marvin_sdk_transaction_test.cpp
    src/marvin_sdk_bridge.cpp
  )
  target_include_directories(marvin_sdk_transaction_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_definitions(marvin_sdk_transaction_test PRIVATE CMPL_LIN)
endif()

ament_package()
