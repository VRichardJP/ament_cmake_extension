cmake_minimum_required(VERSION 3.12)

project(ament_cmake_extension NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_package(
  CONFIG_EXTRAS "ament_cmake_extension-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)