cmake_minimum_required(VERSION 3.10)
project(livox_custommsg_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

add_library(adapter_node SHARED
  src/adapter.cpp
)
ament_target_dependencies(adapter_node rclcpp sensor_msgs std_msgs livox_msgs)

target_compile_options(adapter_node PRIVATE -O3 -Wall -Wextra)

add_executable(livox_custommsg_adapter src/main.cpp)
target_link_libraries(livox_custommsg_adapter adapter_node)
ament_target_dependencies(livox_custommsg_adapter rclcpp sensor_msgs std_msgs livox_msgs)

install(TARGETS adapter_node livox_custommsg_adapter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

ament_package()