cmake_minimum_required(VERSION 3.10)
project(livox_custommsg_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_interfaces REQUIRED)

include_directories(include)

add_executable(livox_custommsg_adapter
  src/adapter.cpp
  src/main.cpp
)
ament_target_dependencies(livox_custommsg_adapter
  rclcpp sensor_msgs std_msgs livox_interfaces
)

target_compile_options(livox_custommsg_adapter PRIVATE -O3 -Wall -Wextra)

install(TARGETS livox_custommsg_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

ament_package()