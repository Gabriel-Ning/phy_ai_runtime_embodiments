cmake_minimum_required(VERSION 3.16)
project(pika_sensor_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
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
