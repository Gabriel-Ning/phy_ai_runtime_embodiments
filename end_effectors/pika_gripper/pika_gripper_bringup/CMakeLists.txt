cmake_minimum_required(VERSION 3.16)
project(pika_gripper_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gripper_slider.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
