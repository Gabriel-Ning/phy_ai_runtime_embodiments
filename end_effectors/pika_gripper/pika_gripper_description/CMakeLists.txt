cmake_minimum_required(VERSION 3.16)
project(pika_gripper_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pika_gripper_description test/test_pika_gripper_description.py)
endif()

install(DIRECTORY config urdf meshes
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
